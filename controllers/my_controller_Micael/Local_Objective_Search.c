#include <Local_Objective_Search.h>


double dO(struct pGA_node** pGA_m, struct pO_node** pO_m, int individual){ // Euclidean distance between individual and obstacles(pO_m) / Eq. (15)
   int cont = (*pO_m)->pO_m.size;
   double f_ed;
   double aux= 1000000000.0; // uma distancia improvavel pro espaco de trabalho
   for (int i=0;i<cont;i++) {
       f_ed = sqrt( pow(((*pGA_m)->pGA_m.X[individual] - (*pO_m)->pO_m.x[i]),2) + pow(((*pGA_m)->pGA_m.Y[individual] -  (*pO_m)->pO_m.y[i]),2) );
       if(f_ed<aux){
           aux = f_ed;
       }
   }

   return aux;

}






double beta(struct pO_linkedlist** pO_list, int m){
    struct pO_node* pO = read_pO_LinkedList(&pO_list, m);
    if(pO->pO_m.size > 0){
        return 1;
    }else {
        return 0;
}

}
double Rd = 1.0;// Radius (Rd) in meters --> sub radius of action
double Z = 1000.0;// (Eq 17)

double cJ(struct pGA_node** pGA_m ,struct pR_linkedlist** pR_list){ // Eq. (17)
    // Funciona assim: A circunferencia atual tem mais chance - a parte da circunferencia atual que tem intersecção com as anteriores ou são anteriores é colocado uma penalidade de 1000
    // dessta forma ele não anda pra tras - e locais já "visitados" não são reavaliados

    //pGA_m --> Nó que contém os indivíduos desta geração analisada
    //pR_list --> linked list que contém a posição atual do robo(ultima) e o historico de todas as posições anteriores
    double e_dist; // euclidean distance
    bool visited_previously = false;

    struct pR_node *aux = (*pR_list)->head;
    for(int i=0; aux->next != NULL; i++){
        aux = aux->next;
        if(aux->next != NULL){
            if( sqrt( pow(((*pGA_m)->pGA_m.X - (*aux).pR_m.X),2) + pow(((*pGA_m)->pGA_m.Y -  (*aux).pR_m.Y),2) ) <= Rd ) {
                visited_previously = true;
            }
        }
     }

    e_dist = sqrt( pow(((*pGA_m)->pGA_m.X - (*aux).pR_m.X),2) + pow(((*pGA_m)->pGA_m.Y -  (*aux).pR_m.Y),2) );

    if(e_dist <= Rd && visited_previously== true){
        return Z;
    } else {
        return 1.0;
    }

     free(aux);
}




double aJ(struct pGA_node** pGA_m, int individual_number, struct node** node_DP, int K_points){ // Eq. (18) -->  dificuldade/modificação --> testar se o ponto pertence ao poligono
    // individual_number --> i-esimo individuo
    // https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon/2922778#2922778

    //nvert: Number of vertices in the polygon. Whether to repeat the first vertex at the end has been discussed in the article referred above.
    //vertx, verty: Arrays containing the x- and y-coordinates of the polygon's vertices.
    //testx, testy: X- and y-coordinate of the test point.

    //int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)

    int nvert = K_points;
    double *vertx = (*node_DP)->delimiting_polygon.Xdp; // AQUI TA ERRADO QUE VC TÀ USANDO o poligono de obstaculo ao inves do DP
    double *verty = (*node_DP)->delimiting_polygon.Ydp; // ver se deu ceto agora
    double testx  = (*pGA_m)->pGA_m.X[individual_number]; // CONFERIR DEPOIS DE IMPLEMENTAR O GA
    double testy  = (*pGA_m)->pGA_m.Y[individual_number];

    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
      if ( ((verty[i]>testy) != (verty[j]>testy)) &&
       (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
         c = !c;
    }
    // --> Quando esta semi-reta intercepta o polígono um número ímpar de vezes, então o ponto está dentro do polígono; caso contrário, ou seja, caso exista um número par de interseções, o ponto está fora
    if(c==1){
        return 0.0;
    }else{
        return INFINITY; // https://stackoverflow.com/questions/23278930/what-is-dbl-max-in-c >>>>> ver isso
    }
}




double gJ(struct pGA_node** pGA_m, struct pO_linkedlist** pO_list, struct pO_node** pO_m, struct pR_linkedlist** pR_list, struct pR_node** pR_m,struct node** node_DP, int m, int individual_number, double final_objective_x, double final_objective_y,int K_points){  // Eq (13)

    double A = dOF_ed( (*pGA_m)->pGA_m.X[individual_number], final_objective_x, (*pGA_m)->pGA_m.Y[individual_number], final_objective_y);
    double B = beta( pO_list , m) * (1/ dO( pGA_m, pO_m ) ); // testar
    double C = beta( pO_list , m) * cJ( pGA_m ,pR_list);
    double D = aJ( pGA_m, individual_number, node_DP,  K_points);

    return A+B+C+D;
}
