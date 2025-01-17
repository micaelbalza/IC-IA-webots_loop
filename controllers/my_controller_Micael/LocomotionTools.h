#ifndef LOCOMOTIONTOOLS  
#define LOCOMOTIONTOOLS

//#include "LinkedList_Detection_of_Obstacles.h"
#include "LinkedList.h"
#include "pR_linkedList.h"
//2
#define MAX_SPEED 2.0
#define NULL_SPEED 0
#define TOLERANCE_FACTOR 0.05

WbDeviceTag left_motor, right_motor; 
WbDeviceTag ds_right_1, ds_right_2, ds_left_1, ds_left_2;
WbDeviceTag compass;
WbDeviceTag GPS;

// Variables and statements------

int K_points = 60;  // points in scan 
int K_points_per_sensor = 60/4;  // points in scan per sensor
// resolution = 90 / number_of_points

int m = 0; // m-ésimo deslocamento, começando em m=0(primeiro deslocamento = ponto inicial)
bool first_m = true; // auxiliar para indicar a primeira vez que é scaneado - Delimiting_Polygon_LinkedList
bool first_n = true; // auxiliar para indicar a primeira vez que é scaneado - pR_LinkedList

/*
* a definição do delimitingPolygonStruct foi para a LinkedList.h
*/


// Variables and statements------



static void step();
static void stop();
static void turn(double angle);
static void scan(int points); 
static void go_forward(double final_position_x, double final_position_y);
static void go_backward(double final_position_x, double final_position_y);
static void wait(double sec);
static void move(double final_position_x, double final_position_y);
static void P_R(); // Eq(1)

#endif