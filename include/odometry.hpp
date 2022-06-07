#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

#include <iostream>

typedef struct{
  double x,y,theta;
} Coordinates;

extern Coordinates position;

void odom(void * ignore);
std::tuple<double, double> ppstep(double path[][2], double currX, double currY, double currTheta, double LFindex);
void ppmove(double path[][2], int lastpointno);

#endif
