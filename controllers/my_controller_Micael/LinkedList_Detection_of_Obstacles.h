#ifndef LINKEDLIST_DETECTION_OF_OBSTACLES 
#define LINKEDLIST_DETECTION_OF_OBSTACLES

#include <stdio.h>
#include <stdlib.h>

/*
* Nescessario somente criar, adicionar, ler e deletar a lista
* OBS: sempre adicionado no fim, e sempre deletado tudo(pois em nossa aplicação não se faz nescessario inserir ordenado, e deletar só alguns nós)
*/



struct delimitingPolygonStruct
 {
   double Xdp[60]; //*4 para cada um dos sensores(ds_right_1_area_scan ... ds_right_2_area_scan ...ds_left_2_area_scan ...)
   double Ydp[60]; //estava 180
 };

 struct node{
    struct delimitingPolygonStruct delimiting_polygon;
    struct node *next;
 };

 struct linkedlist {
     struct node* head;
     int m;
 };

 struct linkedlist* create_LinkedList(){
     struct linkedlist*  newLinkedlist; //allocate linked list
     newLinkedlist = (struct linkedlist*) malloc(sizeof(struct linkedlist));
     if(newLinkedlist==NULL) printf("Linkedlist memory allocation error\n");
     if(newLinkedlist!=NULL) printf("Linkedlist created with address -> %p \n",newLinkedlist);

     newLinkedlist->m = 0;
     newLinkedlist->head = NULL;

     return newLinkedlist;
 }



 void insert_LinkdList(struct linkedlist **list, struct delimitingPolygonStruct delimiting_polygon_m, int *m){
    // m = m-esímo deslocamento
    struct node*  newNode;
    newNode = (struct node*) malloc(sizeof(struct node));
    if(newNode==NULL) printf("Node memory allocated error\n");
    newNode->delimiting_polygon = delimiting_polygon_m;
    newNode->next = NULL;

    if((*list)->head==NULL){ // lista vazia
        (*list)->head = newNode;
    }else{
        struct node* aux = (*list)->head; // percorrer os nós até achar o fim
        while(aux->next != NULL){
            aux = aux->next;
        }
        aux->next = newNode;
    }
    (*m)++; // contabilizando o m-esimo nó

 }

 struct node* read_LinkedList(struct linkedlist** delimiting_polygon_list, int position){
   struct node *aux = (*delimiting_polygon_list)->head;
   for(int i=0; i<position && aux->next != NULL; i++){
     aux = aux->next;
   }
   return aux;
   free(aux);
 }


 void delete_LinkedList(struct linkedlist** delimiting_polygon_list){
   if((*delimiting_polygon_list)->head==NULL){ // lista vazia
       printf("Linked List is empty\n");
   }else{
       struct node* aux1 = (*delimiting_polygon_list)->head;
       struct node* aux2 = NULL;
       while (aux1 != NULL){
         aux2 = aux1;
         aux1 = aux1->next;
         free(aux2);
       }
       free(aux1);
       (*delimiting_polygon_list)->head=NULL;
       (*delimiting_polygon_list)->m=0;
     }
}



 void print_linkedList(struct linkedlist** delimiting_polygon_list, int K_points) {
     if (((*delimiting_polygon_list)->head) != NULL) { // aux percorrendo a lista
         struct node* aux = (*delimiting_polygon_list)->head;
         do {
             printf("___________________________________\n\n");
             for(int i=0;i<K_points; i++){
                 printf("O ponto de (X,Y) do sensor - %3i : (%10f , %10f) \n",i ,   aux->delimiting_polygon.Xdp[i], aux->delimiting_polygon.Ydp[i]);
              }
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
        -->create_LinkedList()    - OK
        -->insert_LinkdList(...)  - OK
        -->print_linkedList(...)  - OK
        -->read_LinkedList(...)   - OK
        -->delete_LinkedList(...) - OK
    CHAMADAS:
        -->struct linkedlist* Delimiting_Polygon_LinkedList = create_LinkedList();
        -->insert_LinkdList(&Delimiting_Polygon_LinkedList, poligono, &Delimiting_Polygon_LinkedList->m);
        -->print_linkedList(&Delimiting_Polygon_LinkedList,180 );  // 180 é k_points
        -->struct node* teste = read_LinkedList(&Delimiting_Polygon_LinkedList, 1); // 1 é o ponto que vc quer ler, iniciando do 0
        -->delete_LinkedList(&Delimiting_Polygon_LinkedList);

    */


#endif