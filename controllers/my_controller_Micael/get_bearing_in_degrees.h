#ifndef GET_BEARING_IN_DEGREES  
#define GET_BEARING_IN_DEGREES

//--------- convert compass to degree
double get_bearing_in_degrees(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[1]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0){
    bearing = bearing + 360.0;
  }
    
  return bearing;
}


double convert_compass_degree_in_cartesian_degree(double degree){
  // webots increasement in clockwise
  // in cartesian, the increasement is in counterclockwise
  
  // and in webots, heading is 0 if robot faced to y axis, we need add +90°, because in cartesian is 0 if faced to X axis;

  degree = 360 - degree;
  degree = degree ;// +90 --> ajuste sincronizado robo
  if(degree > 360.0)
    degree = degree - 360;
  
  return degree;
}

//--------- convert compass to degree -- versão de ajuste para a função turn -- como o restante do cogigo funciona com a outra foi mantida
double get_bearing_in_degrees2(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[1]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  bearing = bearing + 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}




#endif