#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

#include <iostream>

void odom(void * ignore);
std::tuple<double, double> ppstep(double path[][2], double currX, double currY, double currTheta, double LFindex);
void ppmove(double path[][2], int lastpointno);

#endif
