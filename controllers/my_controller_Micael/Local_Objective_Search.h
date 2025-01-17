#ifndef LOCAL_OBJECTIVE_SEARCH 
#define LOCAL_OBJECTIVE_SEARCH

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Detection_of_Obstacles.h>

//-----------------GA definitions/parameters----------------
#define PopulationSize 30

double Rd = 1.0;// Radius (Rd) in meters --> sub radius of action
double Z = 1000.0; // (Eq 17)

//----------------------------------------------------------

bool first_p = true; // auxiliar para indicar a primeira vez que é scaneado - pGA_LinkedList



struct struct_pGA   // Eq. (9)
 {
   double X[PopulationSize];
   double Y[PopulationSize];
 };

 struct pGA_node{
    struct struct_pGA pGA_m; // Eq. (9) --> m-th displacement
    struct pGA_node *next;
 };

 struct pGA_linkedlist {
     struct pGA_node* head;
     int m;
 };

 struct pGA_linkedlist* create_pGA_LinkedList(){
     struct pGA_linkedlist*  new_pGA_Linkedlist; //allocate linked list
     new_pGA_Linkedlist = (struct pGA_linkedlist*) malloc(sizeof(struct pGA_linkedlist));
     if(new_pGA_Linkedlist==NULL) printf("pGA-Linkedlist memory allocation error\n");
     if(new_pGA_Linkedlist!=NULL) printf("pGA-Linkedlist created with address -> %p \n",new_pGA_Linkedlist);

     new_pGA_Linkedlist->m = 0;
     new_pGA_Linkedlist->head = NULL;

     return new_pGA_Linkedlist;
 }



 void insert_pGA_LinkdList(struct pGA_linkedlist **list, struct struct_pGA pGA, int *m){
    // m = m-esímo deslocamento
    struct pGA_node*  new_pga_Node;
    new_pga_Node = (struct pGA_node*) malloc(sizeof(struct pGA_node));
    if(new_pga_Node==NULL) printf("pGA-Node memory allocated error\n");
    new_pga_Node->pGA_m = pGA;
    new_pga_Node->next = NULL;

    if((*list)->head==NULL){ // lista vazia
        (*list)->head = new_pga_Node;
    }else{
        struct pGA_node* aux = (*list)->head; // percorrer os nós até achar o fim
        while(aux->next != NULL){
            aux = aux->next;
        }
        aux->next = new_pga_Node;
    }
    (*m)++; // contabilizando o m-esimo nó

 }

 struct pGA_node* read_pGA_LinkedList(struct pGA_linkedlist** pGA_list, int position){
   struct pGA_node *aux = (*pGA_list)->head;
   for(int i=0; i<position && aux->next != NULL; i++){
     aux = aux->next;
   }
   return aux;
   free(aux);
 }


 void delete_pGA_LinkedList(struct pGA_linkedlist** pGA_list){
   if((*pGA_list)->head==NULL){ // lista vazia
       printf("Linked List - GA is empty\n");
   }else{
       struct pGA_node* aux1 = (*pGA_list)->head;
       struct pGA_node* aux2 = NULL;
       while (aux1 != NULL){
         aux2 = aux1;
         aux1 = aux1->next;
         free(aux2);
       }
       free(aux1);
       (*pGA_list)->head=NULL;
       (*pGA_list)->m=0;
     }
}



 void print_pGA_linkedList(struct pGA_linkedlist** pGA_list) {
     if (((*pGA_list)->head) != NULL) { // aux percorrendo a lista
         struct pGA_node* aux = (*pGA_list)->head;
         int i=0;
         do {
             printf("___________________________________\n\n");

              printf("O ponto (X,Y) encontrado pelo GA no deslocamento - %3i : (%10f , %10f) \n",i ,   aux->pGA_m.X[i], aux->pGA_m.Y[i]);
              i++;
              if (aux != NULL) {
                 printf("___________________________________\n\n");
              }
          } while (aux != NULL);
      } else {
          printf("Linked List is empty\n");
      }
}


 /*

    TESTES:
        -->create_pGA_LinkedList()    - OK
        -->insert_pGA_LinkdList(...)  - OK
        -->print_pR_linkedList(...)  - OK
        -->read_pR_LinkedList(...)   - OK
        -->delete_pR_LinkedList(...) - OK
    CHAMADAS:
        -->struct pGA_linkedlist* pGA_LinkedList = create_pGA_LinkedList();
        -->insert_pGA_LinkdList(&pGA_LinkedList, pGA, &pGA_LinkedList->n);
        -->print_pGA_linkedList(&pGA_LinkedList);
        -->struct pGA_node* teste = read_pGA_LinkedList(&pGA_LinkedList, 1); // 1 é o ponto que vc quer ler, iniciando do 0
        -->delete_pGA_LinkedList(&pGA_LinkedList);

    */





double beta(struct pO_linkedlist** pO_list, int m); // Eq. (16)
double dO(struct pGA_node** pGA_m ,struct pO_node** pO_m ); // Euclidean distance between individual and obstacles(pO_m) / Eq. (15)
double dOF_ed(double xi, double xb, double yi, double yb); // euclidian distance / Eq. (14)
//static double gJ();  // evaluation function  / Eq. (13)
//static void LocalObjectiveSearch();  // etapa. (2.3)

#endif