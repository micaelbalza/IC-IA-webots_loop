#ifndef DETECTION_OF_OBSTACLES  
#define DETECTION_OF_OBSTACLES 

//#include "LocomotionTools.c"
#include <stdio.h>
#include <stdlib.h>

#define Eta 0.005 
//η --> Eta  0 < η <= 0.1   / Eq. (5)

bool first_o = true; // auxiliar para indicar a primeira vez que é scaneado - pR_LinkedList

// An adjustment was made when moving to the MHRTSN strategy, so that there is no longer an eta. is using only the maximum distance from the sensors.
 double r_pv = 3*(1 - Eta); // Eq. (5) --> max distance sesor is 3 #
//double r_pv = 3; // MHRTSN strategy - max distance sesor is 3  max distance sesor is 3 

double F_ed(double xi, double xb, double yi, double yb);  // euclidian distance  / Eq. (8)
void ObstaclesDetection(struct pR_node **pR_m_node, struct node **delimiting_polygon_m_node, int m, int K_points);  // Eq. (7)


 struct struct_pO   // Eq. (6)
  {
    double* x;
    double* y;
    int size;
  };

  struct pO_node{
     struct struct_pO pO_m; // Eq. (6) --> m-th displacement
     struct pO_node *next;
  };

  struct pO_linkedlist {
      struct pO_node* head;
      int m;
  };

  struct pO_linkedlist* create_pO_LinkedList(){
      struct pO_linkedlist*  new_pO_Linkedlist; //allocate linked list
      new_pO_Linkedlist = (struct pO_linkedlist*) malloc(sizeof(struct pO_linkedlist));
      if(new_pO_Linkedlist==NULL) printf("pR-Linkedlist memory allocation error\n");

      new_pO_Linkedlist->m = 0;
      new_pO_Linkedlist->head = NULL;

      return new_pO_Linkedlist;
  }



  void insert_pO_LinkdList(struct pO_linkedlist **list, struct struct_pO pO, int *m){
     // m = m-esímo deslocamento
     struct pO_node*  new_pO_Node;
     new_pO_Node = (struct pO_node*) malloc(sizeof(struct pO_node));
     if(new_pO_Node==NULL) printf("pO-Node memory allocated error\n");
     new_pO_Node->pO_m = pO;
     new_pO_Node->next = NULL;

     if((*list)->head==NULL){ // lista vazia
         (*list)->head = new_pO_Node;
     }else{
         struct pO_node* aux = (*list)->head; // percorrer os nós até achar o fim
         while(aux->next != NULL){
             aux = aux->next;
         }
         aux->next = new_pO_Node;
     }
     (*m)++; // contabilizando o m-esimo nó

  }

  struct pO_node* read_pO_LinkedList(struct pO_linkedlist** pO_list, int position){
    struct pO_node *aux = (*pO_list)->head;
    for(int i=0; i<position && aux->next != NULL; i++){
      aux = aux->next;
    }
    return aux;
    free(aux);
  }


  void delete_pO_LinkedList(struct pO_linkedlist** pO_list){
    if((*pO_list)->head==NULL){ // lista vazia
        printf("Linked List is empty\n");
    }else{
        struct pO_node* aux1 = (*pO_list)->head;
        struct pO_node* aux2 = NULL;
        while (aux1 != NULL){
          aux2 = aux1;
          aux1 = aux1->next;
          free(aux2);
        }
        free(aux1);
        (*pO_list)->head=NULL;
        (*pO_list)->m=0;
      }
 }



  void print_pO_linkedList(struct pO_linkedlist** pO_list) {
      if (((*pO_list)->head) != NULL) { // aux percorrendo a lista
          struct pO_node* aux = (*pO_list)->head;
          do {
              printf("___________________________________\n\n");
              for(int i=0;i<aux->pO_m.size; i++){
                  printf("O ponto de (X,Y) dos obstaculos detectados(pO) - %3i : (%10f , %10f) \n",i ,   aux->pO_m.x[i], aux->pO_m.y[i]);
               }
               aux = aux->next;
               if (aux!= NULL) {
                  printf("___________________________________\n\n");
               }
           } while (aux != NULL);
       } else {
           printf("Linked List is empty\n");
       }
 }


  /*

     TESTES:
         -->create_pO_LinkedList()    - OK
         -->insert_pR_LinkdList(...)  - OK
         -->print_pR_linkedList(...)  - OK
         -->read_pR_LinkedList(...)   - OK
         -->delete_pR_LinkedList(...) - OK
     CHAMADAS:
         -->struct pO_linkedlist* pO_LinkedList = create_pO_LinkedList();
         -->insert_pO_LinkdList(&pO_LinkedList, pO, &pO_LinkedList->m);
         -->print_pO_linkedList(&pO_LinkedList);
         -->struct pO_node* teste = read_pO_LinkedList(&pO_LinkedList, 1); // 1 é o ponto que vc quer ler, iniciando do 0
         -->delete_pO_LinkedList(&pO_LinkedList);

     */
     
#endif
