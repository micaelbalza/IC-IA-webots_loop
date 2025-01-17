#include "Detection_of_Obstacles.h"

struct pO_linkedlist* pO_LinkedList;
int contad = 0;
double F_ed(double xi, double xb, double yi, double yb){  // euclidian distance  / Eq. (8)
  return sqrt( pow((xi-xb),2) + pow((yi-yb),2) );
}

void ObstaclesDetection(struct pR_node **pR_m_node, struct node **delimiting_polygon_m_node, int m, int K_points){  // Eq. (7)
    int cont = 0;
    double auxiliar;
    for (int i=0; i<K_points; i++){
    auxiliar = F_ed((*pR_m_node)->pR_m.X, (*delimiting_polygon_m_node)->delimiting_polygon.Xdp[i],(*pR_m_node)->pR_m.Y, (*delimiting_polygon_m_node)->delimiting_polygon.Ydp[i]);

    if(  auxiliar < r_pv + 0.15 ){
        cont++;
     }
   } 
  double* pO_m_X = (double*) malloc(sizeof(double)*cont);
  double* pO_m_Y = (double*) malloc(sizeof(double)*cont);
  if (first_o == true){ 
     pO_LinkedList = create_pO_LinkedList(); // criando a linked list pO  
  }
  int cont_aux = 0;
  for (int i=0; i<K_points; i++){
    if(  F_ed((*pR_m_node)->pR_m.X, (*delimiting_polygon_m_node)->delimiting_polygon.Xdp[i],(*pR_m_node)->pR_m.Y, (*delimiting_polygon_m_node)->delimiting_polygon.Ydp[i]) < r_pv + 0.15  ){ //VER FORMA MAIS EFICIENTE DE FAZER ISSO
        pO_m_X[cont_aux] = (*delimiting_polygon_m_node)->delimiting_polygon.Xdp[i];
        pO_m_Y[cont_aux] = (*delimiting_polygon_m_node)->delimiting_polygon.Ydp[i];
        cont_aux++;
     }
   } 
  struct struct_pO pO_m;
  pO_m.x = pO_m_X;
  pO_m.y = pO_m_Y;
  pO_m.size = cont;
  insert_pO_LinkdList(&pO_LinkedList, pO_m, &pO_LinkedList->m);
}
