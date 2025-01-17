/*
 * File:          my_controller_Micael.c
 * Date:
 * Description:
 * Author: Micael Balza ( micaelbalza@hotmail.com )
 * Modifications:
 */



/* 
* Atualmente está configurado para capturar 180 pontos(K_points) no scan, atribuindo então 45 pontos a cada sensor
* ou seja, a cada 2° eu pego um ponto -> pode ser ajustado em LocomotionTools.h e LocomotionTools.c 
*/


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h> // acionamento motor
#include <webots/distance_sensor.h>  // acionamento sensor de distancia
#include <webots/compass.h> // bússola
#include <webots/gps.h> // GPS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h> // declaracao para usar pThread
#include <LocomotionTools.c>
#include <Detection_of_Obstacles.c>
#include <save_txt.c>
#include <read_txt.c>

#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#define MAX 40
#define PORT 12345
#define SA struct sockaddr

#include <string.h>
#define  BUFSIZE 256

#define TIME_STEP 64 

#define OBJECTIVE_TOLERANCE_FACTOR 0.3 
// estava 0.2


struct tcp_parameters
{
    int confirmed_unlock, sockfd;
    WbMutexRef TCP_mutex_thread;
};

struct tcp_parameters structTCPParameters;



int main(int argc, char **argv) {

    // Acessa o valor da variável de ambiente DESTINATION_FOLDER
    char *destination_folder = getenv("DESTINATION_FOLDER");

    // Verifica se a variável de ambiente existe e possui um valor
    if (destination_folder != NULL) {
        printf("O caminho da pasta fornecido é: %s\n", destination_folder);
    } else {
        printf("A variável de ambiente DESTINATION_FOLDER não está definida.\n");
    }
    
    // Acessa o valor da variável de ambiente END_POINT
    char *end_point = getenv("END_POINT");
    // Verifica se a variável de ambiente existe e possui um valor
    if (end_point == NULL) {
        printf("A variável de ambiente END_POINT não está definida.\n");
    }

    /* necessary to initialize webots stuff */
    wb_robot_init();

    structTCPParameters.confirmed_unlock = 0; 
    structTCPParameters.TCP_mutex_thread = wb_robot_mutex_new();

    void *ptr = &structTCPParameters;
  
   
   // Initialize motor
   left_motor  = wb_robot_get_device("motor1");
   right_motor = wb_robot_get_device("motor2");
   
   wb_motor_set_position(left_motor, INFINITY);
   wb_motor_set_position(right_motor, INFINITY);
   
   wb_motor_set_velocity(left_motor, 0.0);
   wb_motor_set_velocity(right_motor, 0.0);
   
   // Initialize Distance sensor
   // number 1 = front
   ds_right_1 = wb_robot_get_device("distance sensor_D_1");
   ds_right_2 = wb_robot_get_device("distance sensor_D_2");
   ds_left_1 = wb_robot_get_device("distance sensor_E_1");
   ds_left_2 = wb_robot_get_device("distance sensor_E_2");
   
   wb_distance_sensor_enable(ds_right_1, TIME_STEP);
   wb_distance_sensor_enable(ds_right_2, TIME_STEP);
   wb_distance_sensor_enable(ds_left_1, TIME_STEP);
   wb_distance_sensor_enable(ds_left_2, TIME_STEP);
   
   // Initialize compass sensor
   compass = wb_robot_get_device("compass_sensor");
   
   wb_compass_enable(compass, TIME_STEP);
   
   
   // Initialize GPS sensor
   GPS = wb_robot_get_device("gps_sensor");
   
   wb_gps_enable(GPS, TIME_STEP);
   
   
   // inicialize thread
   void *TCP_thread(void *arg);
   wb_robot_task_new(TCP_thread, ptr);
   
   int m_displacement = 0; // iteração atual

   int saved_successfully;
   
   double local_objective_x;
   double local_objective_y;
   
   // Divide a string nas coordenadas x e y
   char *token;
   double final_objective[2];
   token = strtok(end_point, ",");
   final_objective[0] = atof(token); // converte a string para double
   token = strtok(NULL, ",");
   final_objective[1] = atof(token); // converte a string para double
   
   //-------------------------------------------------------
   //double final_objective[2] = {-4.0, 2.5}; // Caso - 1   
   //double final_objective[2] = {2.0, 0.0};  // Caso - 2   (A2)
   //double final_objective[2] = {4.0, 4.0};    // Caso - 3   (A3) (SX)
   //double final_objective[2] = {2.0, -2.0}; // Caso - 4       (A1)
   //-------------------------------------------------------
   // caso 1 é usado nos exemplos de tempo
    //double final_objective[2] = {-3.0, 0.0};  // Caso - 2 dinamico (A4)
   //-------------------------------------------------------

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     
      const double *a = wb_gps_get_values(GPS);
      double position_x = a[0];
      double position_y = a[1];
      
      if(sqrt( pow((final_objective[0] - position_x) , 2) + pow((final_objective[1] - position_y), 2) ) <= OBJECTIVE_TOLERANCE_FACTOR){
       printf("chegou no objetivo!!!");
       break;
     }
     
      P_R(); // Eq(1)
       
      printf("M-displacement - %i \n",m_displacement);
           
      scan(K_points_per_sensor); // eq(4)
      struct pR_node* pR_displacement = read_pR_LinkedList(&pR_LinkedList, m_displacement); // 1 é o ponto que vc quer ler, iniciando do 0
      struct node* pDP_displacement = read_LinkedList(&Delimiting_Polygon_LinkedList, m_displacement); // 1 é o ponto que vc quer ler, iniciando do 0
      ObstaclesDetection(&pR_displacement, &pDP_displacement, m_displacement, K_points);  // Eq. (7)  //// Obs, não está usando o m_displacement
      struct pO_node* pO_displacement = read_pO_LinkedList(&pO_LinkedList, m_displacement); // 1 é o ponto que vc quer ler, iniciando do 0
      saved_successfully = save_txt(pR_displacement, pDP_displacement, pO_displacement, m_displacement, K_points, final_objective, destination_folder);
      
      if(saved_successfully != 0){
        printf("Falha ao salvar o txt \n");
      } 
      
      wb_robot_mutex_lock(structTCPParameters.TCP_mutex_thread);
      read_txt( m_displacement, &local_objective_x, &local_objective_y, destination_folder);
      structTCPParameters.confirmed_unlock = 1;
      wb_robot_mutex_unlock(structTCPParameters.TCP_mutex_thread);
   
      move( local_objective_x , local_objective_y );
      
      m_displacement++; 
        
        if( m_displacement >= 1000 ){
       break;
     }
     
     step();
      /* Process sensor data here */
  }

  /* Enter your cleanup code here */
  close(structTCPParameters.sockfd);
  wb_robot_mutex_delete(structTCPParameters.TCP_mutex_thread);
  
 
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

void func(int connfd, struct tcp_parameters* targ)
{    
    char buff[MAX];
        
    // infinite loop for chat
    while (1) {
        bzero(buff, MAX);
   
        // read the message from client and copy it in buffer
        read(connfd, buff, sizeof(buff));
        // read buffer and compare the message
        if (strncmp("read", buff, 4) == 0) {
            //printf("pode ler...+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

       

            wb_robot_mutex_unlock(targ->TCP_mutex_thread);//NOVA PIZZA
            
            
       
            while (1){
            
              printf("targ->confirmed_unlock...++++++++++++++++++++++++++++++++|  %d  - end --> %x  |+++++++++++++++++++++++++++++++\n",targ->confirmed_unlock, &targ->confirmed_unlock);
              if (targ->confirmed_unlock == 1){
                printf("TARG CONFIRMADO...+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
                targ->confirmed_unlock = 0;
                send(connfd, "received", 8, 0); 
                
                printf("ENVIADO...+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
                break;
                
              }
              
              //sleep(0.1);
              
            }
            wb_robot_mutex_lock(targ->TCP_mutex_thread);
               
        }
        
        // if msg contains "exit" then server exit and chat ended.
        if (strncmp("exit", buff, 4) == 0) {
            printf("Server Exit...\n");
            wb_robot_mutex_unlock(targ->TCP_mutex_thread);
            sleep(0.5);
            wb_robot_mutex_delete(targ->TCP_mutex_thread);
            break;
        }
    }
}



void *TCP_thread(void *arg){  // referencia de onde pegou https://www.geeksforgeeks.org/tcp-server-client-implementation-in-c/
    
    struct tcp_parameters* targ = (struct tcp_parameters *)arg;
    
    wb_robot_mutex_lock((struct tcp_parameters*)targ->TCP_mutex_thread); // na primeira vez trava o semaforo quando acontecer uma conexao
   
    //int sockfd;
    int connfd, len;
    struct sockaddr_in servaddr, cli;
   
    // socket create and verification
    //sockfd = socket(AF_INET, SOCK_STREAM, 0);
    targ->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    
    if ((struct tcp_parameters*)targ->sockfd == -1) {
        printf("socket creation failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully created..\n");
    bzero(&servaddr, sizeof(servaddr));
   
    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
   
    // Binding newly created socket to given IP and verification
    if ((bind((struct tcp_parameters*)targ->sockfd, (SA*)&servaddr, sizeof(servaddr))) != 0) {
        printf("socket bind failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully binded..\n");
   
    // Now server is ready to listen and verification
    if ((listen((struct tcp_parameters*)targ->sockfd, 5)) != 0) {
        printf("Listen failed...\n");
        exit(0);
    }
    else
        printf("Server listening..\n");
    len = sizeof(cli);
   
    // Accept the data packet from client and verification
    connfd = accept((struct tcp_parameters*)targ->sockfd, (SA*)&cli, &len);
    if (connfd < 0) {
        printf("server accept failed...\n");
        exit(0);
    }
    else
        printf("server accept the client...\n");
        
    // Function for chatting between client and server
    func(connfd,targ);
   
    // After chatting close the socket
    close((struct tcp_parameters*)targ->sockfd); 
}