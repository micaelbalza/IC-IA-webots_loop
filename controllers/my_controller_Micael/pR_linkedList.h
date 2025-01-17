#ifndef PR_LINKEDLIST  
#define PR_LINKEDLIST

#include <stdio.h>
#include <stdlib.h>

/*
* Nescessario somente criar, adicionar, ler e deletar a lista
* OBS: sempre adicionado no fim, e sempre deletado tudo(pois em nossa aplicação não se faz nescessario inserir ordenado, e deletar só alguns nós)
*/



struct struct_pR   // Eq. (1)
 {
   double X; 
   double Y;
 };

 struct pR_node{
    struct struct_pR pR_m; // Eq. (1) --> m-th displacement
    struct pR_node *next;
 };

 struct pR_linkedlist {
     struct pR_node* head;
     int n;
 };

 struct pR_linkedlist* create_pR_LinkedList(){
     struct pR_linkedlist*  new_pR_Linkedlist; //allocate linked list
     new_pR_Linkedlist = (struct pR_linkedlist*) malloc(sizeof(struct pR_linkedlist));
     if(new_pR_Linkedlist==NULL) printf("pR-Linkedlist memory allocation error\n");
     if(new_pR_Linkedlist!=NULL) printf("pR-Linkedlist created with address -> %p \n",new_pR_Linkedlist);

     new_pR_Linkedlist->n = 0;
     new_pR_Linkedlist->head = NULL;

     return new_pR_Linkedlist;
 }



 void insert_pR_LinkdList(struct pR_linkedlist **list, struct struct_pR pR, int *n){
    // m = m-esímo deslocamento
    struct pR_node*  new_pr_Node;
    new_pr_Node = (struct pR_node*) malloc(sizeof(struct pR_node));
    if(new_pr_Node==NULL) printf("pR-Node memory allocated error\n");
    new_pr_Node->pR_m = pR;
    new_pr_Node->next = NULL;

    if((*list)->head==NULL){ // lista vazia
        (*list)->head = new_pr_Node;
    }else{
        struct pR_node* aux = (*list)->head; // percorrer os nós até achar o fim
        while(aux->next != NULL){
            aux = aux->next;
        }
        aux->next = new_pr_Node;
    }
    (*n)++; // contabilizando o m-esimo nó

 }

 struct pR_node* read_pR_LinkedList(struct pR_linkedlist** pR_list, int position){
   struct pR_node *aux = (*pR_list)->head;
   for(int i=0; i<position && aux->next != NULL; i++){
     aux = aux->next;
   }
   return aux;
   free(aux);
 }


 void delete_pR_LinkedList(struct pR_linkedlist** pR_list){
   if((*pR_list)->head==NULL){ // lista vazia
       printf("Linked List is empty\n");
   }else{
       struct pR_node* aux1 = (*pR_list)->head;
       struct pR_node* aux2 = NULL;
       while (aux1 != NULL){
         aux2 = aux1;
         aux1 = aux1->next;
         free(aux2);
       }
       free(aux1);
       (*pR_list)->head=NULL;
       (*pR_list)->n=0;
     }
}



 void print_pR_linkedList(struct pR_linkedlist** pR_list) {
     if (((*pR_list)->head) != NULL) { // aux percorrendo a lista
         struct pR_node* aux = (*pR_list)->head;
         int i=0;
         do {
             printf("___________________________________\n\n");
             
              printf("A localização de (X,Y) do robo no deslocamento - %3i : (%10f , %10f) \n",i ,   aux->pR_m.X, aux->pR_m.Y);
              i++;
              aux = aux->next;
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
        -->create_pR_LinkedList()    - OK
        -->insert_pR_LinkdList(...)  - OK
        -->print_pR_linkedList(...)  - OK
        -->read_pR_LinkedList(...)   - OK
        -->delete_pR_LinkedList(...) - OK
    CHAMADAS:
        -->struct pR_linkedlist* pR_LinkedList = create_pR_LinkedList();
        -->insert_pR_LinkdList(&pR_LinkedList, pR, &pR_LinkedList->n);
        -->print_pR_linkedList(&pR_LinkedList);
        -->struct pR_node* teste = read_pR_LinkedList(&pR_LinkedList, 1); // 1 é o ponto que vc quer ler, iniciando do 0
        -->delete_pR_LinkedList(&pR_LinkedList);

    */
#endif