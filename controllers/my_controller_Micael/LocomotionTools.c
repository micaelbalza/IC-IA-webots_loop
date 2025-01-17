#include <math.h>
#include "LocomotionTools.h"
#include <get_bearing_in_degrees.h>

#define PI 3.14159265

struct linkedlist* Delimiting_Polygon_LinkedList;
struct pR_linkedlist* pR_LinkedList;

/* helper functions */

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}



static void stop() {
  wb_motor_set_velocity(left_motor, NULL_SPEED);
  wb_motor_set_velocity(right_motor, NULL_SPEED);
}



static void turn(double angle) { // -360 <= angle <= +360
  stop();
  double starting_angle = get_bearing_in_degrees2(compass); //leitura angulo inicial
  //printf("starting_angle: %f \n",starting_angle);
  double start_end_angle;
  bool flag = false;
  if(angle < 0.0){ // Quero virar a direita
      //---------------------- caso 0 ------------------------ 
      if(starting_angle + angle < 0.0){ //passa pelo 0
        start_end_angle = starting_angle + angle + 360; // ex: 10 - 20 = 350
        step();
        wb_motor_set_velocity(left_motor,  MAX_SPEED*0.2);
        wb_motor_set_velocity(right_motor,-MAX_SPEED*0.2);
        double compass_degree;  // angulo medido
        while(true) {
          compass_degree = get_bearing_in_degrees2(compass);
          step();
          if(compass_degree >= start_end_angle)  flag = true; // passou
          if(flag == true &&  compass_degree <= start_end_angle){ //se passou pelo 0 e ang_medido<= quero ir: pare
            break;
          } 
        } 
        stop();
        step();
        //----------------------FIM caso 0 --------------------- 
        
        //---------------------- caso 1 ------------------------
        }else{ // não passou pelo 0
          start_end_angle = starting_angle + angle; // ex: 90 - 30 = 60
          //printf("2-start_end_angle : %f\n",start_end_angle);
          step();
          wb_motor_set_velocity(left_motor,  MAX_SPEED*0.2);
          wb_motor_set_velocity(right_motor,-MAX_SPEED*0.2);
          double compass_degree;  // angulo medido
          while(true) {
            compass_degree = get_bearing_in_degrees2(compass);

            step();
            if(compass_degree <= start_end_angle){ //se passou pelo 0 e ang_medido<= quero ir: pare
              break;
            } 
          }
          stop();
          step();
        }
        //----------------------FIM caso 1 --------------------- 
    
  }else{ // Quero virar a esquerda
     
      //---------------------- caso 2 ------------------------ 
      if(starting_angle + angle > 360){ //passa pelo 0
        start_end_angle = starting_angle + angle - 360; // ex: 350 + 20 = 10
        step();
        wb_motor_set_velocity(left_motor, -MAX_SPEED*0.2);
        wb_motor_set_velocity(right_motor, MAX_SPEED*0.2);
        double compass_degree;  // angulo medido
        while(true) {
          compass_degree = get_bearing_in_degrees2(compass);
          //printf("3-compass_degree : %f\n",compass_degree);
          step();
          if(compass_degree <= start_end_angle)  flag = true; // passou
          if(flag == true &&  compass_degree >= start_end_angle){ //se passou pelo 0 e ang_medido>= onde quero ir: pare
            break;
          } 
        } 
        stop();
        step();
        //----------------------FIM caso 2 --------------------- 
  
        //---------------------- caso 3 ------------------------
        }else{ // não passou pelo 0
          start_end_angle = starting_angle + angle; // ex: 90 + 30 = 120
          step();
          wb_motor_set_velocity(left_motor, -MAX_SPEED*0.2);
          wb_motor_set_velocity(right_motor, MAX_SPEED*0.2);
          double compass_degree;  // angulo medido
          while(true) {
            compass_degree = get_bearing_in_degrees2(compass);
            //printf("4-compass_degree : %f\n",compass_degree);
            step();
            if(compass_degree >= start_end_angle){ //se passou pelo 0 e ang_medido<= quero ir: pare
              break;
            } 
          } 
          stop();
          step();  
        }
        //----------------------FIM caso 3 ---------------------         
   }
}


static void scan(int points) { 

  stop();
  double starting_angle = get_bearing_in_degrees(compass) ; // angulo inicial
  double start_end_angle; // final angle
  
  
  const double *a = wb_gps_get_values(GPS); // position
  double position_x,position_y;
  position_x = a[0];
  position_y = a[1];

  
  //as we have 4 sensors, a 90° turn analyzes the 360°
  double resolution = 90.0 / points;
  double vector_distance_right_1_area_scan[points];
  //double angle__distance_right_1_area_scan[points];
  double vector_distance_right_2_area_scan[points];
  //double angle__distance_right_2_area_scan[points];
  double vector_distance_left_1_area_scan[points];
  //double angle__distance_left_1_area_scan[points];
  double vector_distance_left_2_area_scan[points];
  double angle__distance_sensor[points];
    //---------------------- scan caso 0 ------------------------ 
  if(starting_angle + 90 > 360){ //passa pelo 0
    
    start_end_angle = starting_angle + 90 - 360; // ex: 350 + 20 = 10
    double compass_degree;// angulo medido
    int i = 0;
    bool flag = false;
    step();
    wb_motor_set_velocity(left_motor, -MAX_SPEED*0.2);
    wb_motor_set_velocity(right_motor, MAX_SPEED*0.2);
    while(true) {
          compass_degree = get_bearing_in_degrees(compass);          
          if((compass_degree >= i*resolution+starting_angle) && (compass_degree < (i+1)*resolution + starting_angle) && flag == false){ // antes de passar pelo 0
              vector_distance_right_1_area_scan[i] = wb_distance_sensor_get_value(ds_right_1);
              vector_distance_right_2_area_scan[i] = wb_distance_sensor_get_value(ds_right_2);
              vector_distance_left_1_area_scan[i]  = wb_distance_sensor_get_value(ds_left_1);
              vector_distance_left_2_area_scan[i]  = wb_distance_sensor_get_value(ds_left_2);
              compass_degree = get_bearing_in_degrees(compass);
              angle__distance_sensor[i] = compass_degree;
              i++;
              if(i>=points) break;
          }else if((compass_degree+360 >= i*resolution+starting_angle) && (compass_degree+360 < (i+1)*resolution + starting_angle)){ // depois de passar pelo 0
              vector_distance_right_1_area_scan[i] = wb_distance_sensor_get_value(ds_right_1);
              vector_distance_right_2_area_scan[i] = wb_distance_sensor_get_value(ds_right_2);
              vector_distance_left_1_area_scan[i]  = wb_distance_sensor_get_value(ds_left_1);
              vector_distance_left_2_area_scan[i]  = wb_distance_sensor_get_value(ds_left_2);
              angle__distance_sensor[i] = compass_degree;
              i++;
              if(i>=points) break;
          }
          
            step();
            if(compass_degree <= start_end_angle)  flag = true; // passou(flag do 0)
            if(flag == true &&  compass_degree >= start_end_angle){ //se passou pelo 0 e ang_medido>= onde quero ir: pare
              break;
            } 
          } 
    stop();
    step();
    //----------------------FIM scan caso 0 --------------------- 

    //---------------------- scan caso 1 ------------------------
    }else{ // não passou pelo 0
      start_end_angle = starting_angle + 90; // ex: 90 + 30 = 120
      double compass_degree;// angulo medido
      int i = 0;
      step();
      wb_motor_set_velocity(left_motor, -MAX_SPEED*0.2);
      wb_motor_set_velocity(right_motor, MAX_SPEED*0.2);
      while(true) {
            compass_degree = get_bearing_in_degrees(compass);
            if((compass_degree >= i*resolution+starting_angle) && (compass_degree < (i+1)*resolution + starting_angle)){
                vector_distance_right_1_area_scan[i] = wb_distance_sensor_get_value(ds_right_1);
                vector_distance_right_2_area_scan[i] = wb_distance_sensor_get_value(ds_right_2);
                vector_distance_left_1_area_scan[i]  = wb_distance_sensor_get_value(ds_left_1);
                vector_distance_left_2_area_scan[i]  = wb_distance_sensor_get_value(ds_left_2);
                angle__distance_sensor[i] = compass_degree;
                i++;
                if(i>=points) break;

            }
            step();
            if(compass_degree >= start_end_angle){ //se passou pelo 0 e ang_medido<= quero ir: pare
                break;
            } 
      }
      stop();
      step();  
    }
    //----------------------FIM scan caso 1 --------------------- 

  if (first_m == true){ 
    Delimiting_Polygon_LinkedList = create_LinkedList(); // criando a linked list Delimiting_Polygon
  }
  
  /*    Now we calculate the x and y values ​​of the polygon
        Xdp = position_x(robot) + cos(theta)*(distanceFromCenterToSensor + measured distance);
        Ydp = position_y(robot) + sin(theta)*(distanceFromCenterToSensor + measured distance;
        theta = compass_degree - +45°
          +45° because of sesor orientation with robotprintf("scan-----1\n");
        add 0.15 on sensor due to robot diameter (sensor configuration) 
  */
  //double aux = 3.14159265 / 180; //aux to convert degree in radian
  
  struct delimitingPolygonStruct delimiting_polygon;
  double aux = PI / 180 ;
  double distanceFromCenterToSensor = 0.15;
  for(int i=0; i<points; i++){
  
    delimiting_polygon.Xdp[i] = position_x + (distanceFromCenterToSensor + vector_distance_right_1_area_scan[i]) * cos((angle__distance_sensor[i]+ 180 + 315) * aux); //315° --> sensor R1
    delimiting_polygon.Ydp[i] = position_y + (distanceFromCenterToSensor + vector_distance_right_1_area_scan[i]) * sin((angle__distance_sensor[i]+ 180 + 315 )* aux);
    
    delimiting_polygon.Xdp[i + K_points_per_sensor] = position_x + (distanceFromCenterToSensor + vector_distance_left_1_area_scan[i]) * cos((angle__distance_sensor[i]+ 180 + 45) * aux); //45° --> sensor D1
    delimiting_polygon.Ydp[i + K_points_per_sensor] = position_y + (distanceFromCenterToSensor + vector_distance_left_1_area_scan[i]) * sin((angle__distance_sensor[i]+ 180 + 45) * aux);
   
    delimiting_polygon.Xdp[i + K_points_per_sensor*2] = position_x + (distanceFromCenterToSensor + vector_distance_left_2_area_scan[i]) * cos((angle__distance_sensor[i]+ 180 + 135) * aux); //135° --> sensor D2
    delimiting_polygon.Ydp[i + K_points_per_sensor*2] = position_y + (distanceFromCenterToSensor + vector_distance_left_2_area_scan[i]) * sin((angle__distance_sensor[i]+ 180 + 135) * aux); 
    
    delimiting_polygon.Xdp[i + K_points_per_sensor*3] = position_x + (distanceFromCenterToSensor + vector_distance_right_2_area_scan[i]) * cos((angle__distance_sensor[i]+ 180 + 225) * aux); //225° --> sensor R2
    delimiting_polygon.Ydp[i + K_points_per_sensor*3] = position_y + (distanceFromCenterToSensor + vector_distance_right_2_area_scan[i]) * sin((angle__distance_sensor[i]+ 180 + 225 )* aux);
   
    
    
  }
  insert_LinkdList(&Delimiting_Polygon_LinkedList, delimiting_polygon, &Delimiting_Polygon_LinkedList->m); // Inserir o poligono na linked list
  first_m = false;
  // ---return to position
  turn(-90);
  // ---end return to position

}



static void go_forward(double final_position_x, double final_position_y) { 
  
  //double error = 0.05;
  const double *a = wb_gps_get_values(GPS);
  double position_x,position_y;
  double euclidean_distance = 100000.0; //um valor inicial bem grande
  
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  while(true) {
    a = wb_gps_get_values(GPS);
    position_x = a[0];
    position_y = a[1];
    euclidean_distance = sqrt( pow((final_position_x - position_x) , 2) + pow((final_position_y - position_y), 2) );
    if( euclidean_distance <= TOLERANCE_FACTOR ){
      break;
    } 
    step();
  }
  stop();
  step();
}

static void go_backward(double final_position_x, double final_position_y) {
  const double *a = wb_gps_get_values(GPS);
  double position_x,position_y;
  double euclidean_distance = 100000.0; //um valor inicial bem grande
  
  wb_motor_set_velocity(left_motor,  -MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  
  while(true) {
    a = wb_gps_get_values(GPS);
    position_x = a[0];
    position_y = a[1];
    euclidean_distance = sqrt( pow((final_position_x - position_x) , 2) + pow((final_position_y - position_y), 2) );
    if( euclidean_distance <= TOLERANCE_FACTOR ){
      break;
    } 
    step();
  }
  stop();
  step();
}

static void wait(double sec) {
  double start_t = wb_robot_get_time();
  do {
    step();
  } while (start_t + sec > wb_robot_get_time());
}


static void move(double final_position_x, double final_position_y) {
  // three steps:
  // if equal -> turn -> go 
  const double *a = wb_gps_get_values(GPS);
  double position_x = a[0];
  double position_y = a[1];
  
  double euclidean_distance = sqrt( pow((final_position_x - position_x) , 2) + pow((final_position_y - position_y), 2) );
  if(euclidean_distance <= TOLERANCE_FACTOR) // equal cordinate
    return;
  
  
  // Turn
  double theta_in_degrees = atan2(final_position_y - position_y, final_position_x - position_x) * 180.0 / M_PI; // convert arctg2(radians) in degrees
  double starting_angle = get_bearing_in_degrees2(compass); //leitura angulo inicial
  double convert_starting_angle = get_bearing_in_degrees2(compass);
  
  if(convert_starting_angle > 180.0 ){
    convert_starting_angle = convert_starting_angle - 360.0;
  }

  if(theta_in_degrees == convert_starting_angle){
    //done
  }else if(theta_in_degrees > 0 &&  convert_starting_angle > 0) {  
      if(theta_in_degrees > convert_starting_angle){
        // left turn
        turn((theta_in_degrees - convert_starting_angle));
      }else{
        // right turn
        // theta_in_degrees < convert_starting_angle
        turn(theta_in_degrees - convert_starting_angle);
      } 
  }else if(theta_in_degrees < 0 &&  convert_starting_angle < 0){ 
      if(theta_in_degrees < convert_starting_angle){
        // right turn 
        turn((theta_in_degrees - convert_starting_angle));
      }else{
        // left turn
        // theta_in_degrees > convert_starting_angle
        turn((theta_in_degrees - convert_starting_angle));
      } 
  }else if(theta_in_degrees>=0){
      if((convert_starting_angle > theta_in_degrees-180) && (convert_starting_angle <= theta_in_degrees)){
        // left turn
        turn(-((starting_angle-theta_in_degrees)-360));
      }else{
        // right turn
        turn(theta_in_degrees  - starting_angle);
      }
  
  }else { // theta_in_degrees < 0
      if((theta_in_degrees+360 > starting_angle ) && (theta_in_degrees+180 < starting_angle)){
        // left turn
        turn((theta_in_degrees+360)-starting_angle);
      }else{
        // right turn
        turn(-(starting_angle-theta_in_degrees));
      }
  
  }
  
  //Move
  go_forward(final_position_x,final_position_y);
  
}

static void P_R(){ // Eq(1)
  if (first_n == true){ 
    pR_LinkedList = create_pR_LinkedList(); // criando a linked list pR
  }
  first_n = false;
  const double *a = wb_gps_get_values(GPS);
  struct struct_pR pR_m;
  pR_m.X=a[0];
  pR_m.Y=a[1];
  insert_pR_LinkdList(&pR_LinkedList, pR_m, &pR_LinkedList->n);
} 









