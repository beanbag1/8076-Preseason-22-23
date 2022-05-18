//everything is in inches unless otherwise stated

#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "odometry.hpp"
#include <iostream>
#include <algorithm>
using namespace okapi;
using namespace std;

//declare base stuff
auto trackerL = ADIEncoder('C', 'D', true);
auto trackerR = ADIEncoder('A', 'B', true);

//define FIXED values
#define sL 7.75
#define sR 7.75
#define sTotal 15.5
#define inchesperdeg (2.75*pi/360)

//define CHANGING values
double lastTheta = 0; //starting orientation
double currLeft = 0;
double currRight = 0;

//struct
typedef struct{
  double x,y,theta;
} Coordinates;

Coordinates position;

void odom(void * ignore) {
  trackerL.reset();
  trackerR.reset();
  double prevLeft = 0;
  double prevRight = 0;
  double prevTheta = 0;
  while(true){
    currLeft = trackerL.get();
    currRight = trackerR.get();
    double changeLeft = (currLeft - prevLeft)*inchesperdeg; //both are alr converted to inches
    double changeRight = (currRight - prevRight)*inchesperdeg;
    position.theta = (lastTheta + (currLeft - currRight)*inchesperdeg/sTotal); //converts radians to degrees
    if (position.theta >= (2*pi)) {
      position.theta = position.theta - 2*pi;
    } else if (position.theta <= 0) {
      position.theta = position.theta + 2*pi;
    }
    double changeTheta = position.theta - prevTheta;
    double sumofChange = changeLeft + changeRight;
    if (changeTheta == 0) { //if changeTheta = 0 -> robot is forward or back purely
      position.x = position.x + (sumofChange/2*sin(position.theta));
      position.y = position.y + (sumofChange/2*cos(position.theta));
    } else {
      double localYchange = (sumofChange/changeTheta)*sin(changeTheta/2);
      position.x = position.x + (localYchange*sin(prevTheta+(changeTheta/2)));
      position.y = position.y + (localYchange*cos(prevTheta+(changeTheta/2)));
    }
    prevLeft = currLeft;
    prevRight = currRight;
    prevTheta = position.theta; //setting prev variables

    //print to brain screen
    pros::screen::print(TEXT_LARGE, 1, "X-coordinate: %f", position.x);
    pros::screen::print(TEXT_LARGE, 3, "Y-coordinate: %f", position.y);
    pros::screen::print(TEXT_LARGE, 5, "Theta: %f", position.theta*180/pi);
    //print to console
    //std::cout << position.x << " " << position.y << " " << position.theta << std::endl;

    pros::Task::delay(5); //wait 5ms before running again
  }
}

double sign(double num){
  if (num >= 0) return 1;
  else return -1;
}

double ptpdist(double p1[], double p2[]){
  double dist = sqrt((p2[0] - p1[0])*(p2[0] - p1[0]) + (p2[1] - p1[1])*(p2[1] - p1[1]));
  return dist;
}

tuple<double, double> ppstep(double path[][2], double LFindex) {
  bool intersectFound = false;
  double currX = position.x;
  double currY = position.y;
  double currTheta = position.theta;
  double goalp[] = {};
  double lookaheaddist = 0.001;
  int lastfoundindex = LFindex;
  int startingindex = lastfoundindex;
  double currentpos[] = {currX, currY};
  int i;
  for(i = startingindex; i < (sizeof(path)/sizeof(path[0])-1);) {
    double x1 = path[i][0] - currX;
    double y1 = path[i][1] - currY;
    double x2 = path[i+1][0] - currX;
    double y2 = path[i+1][1] - currY;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = sqrt(dx*dx + dy*dy);
    double d = x1*y2 - x2*y1;
    double discriminant = (lookaheaddist*lookaheaddist)*(dr*dr) - d*d;
    if (discriminant >= 0) {
      double solx1 = (d*dy + sign(dy)*dx*sqrt(discriminant))/(dr*dr);
      double solx2 = (d*dy - sign(dy)*dx*sqrt(discriminant))/(dr*dr);
      double soly1 = (-d*dx + abs(dy)*sqrt(discriminant))/(dr*dr);
      double soly2 = (-d*dx - abs(dy)*sqrt(discriminant))/(dr*dr);
      double solp1[] = {(solx1 + currX), (soly1 + currY)};
      double solp2[] = {(solx2 + currX), (soly2 + currY)};
      double minX = min(path[i][0], path[i+1][0]);
      double maxX = max(path[i][0], path[i+1][0]);
      double minY = min(path[i][1], path[i+1][1]);
      double maxY = max(path[i][1], path[i+1][1]);
      if (((minX <= solp1[0] <= maxX) && (minY <= solp1[1] <= maxY)) || ((minX <= solp2[0] <= maxX) && (minY <= solp2[1] <= maxY))) {
        intersectFound = true;
        if (((minX <= solp1[0] <= maxX) && (minY <= solp1[1] <= maxY)) && ((minX <= solp2[0] <= maxX) && (minY <= solp2[1] <= maxY))) {
          if (ptpdist(solp1, path[i+1]) < ptpdist(solp2, path[i+1])) {
            goalp[0] = solp1[0];
            goalp[1] = solp1[1];
          }
          else {
            goalp[0] = solp2[0];
            goalp[1] = solp2[1];
          }
        }
        else {
          if ((minX <= solp1[0] <= maxX) && (minY <= solp1[1] <= maxY)) {
            goalp[0] = solp1[0];
            goalp[1] = solp1[1];
          }
          else {
            goalp[0] = solp2[0];
            goalp[1] = solp2[1];
          }
        }
        if (ptpdist(goalp, path[i+1]) < ptpdist(currentpos, path[i+1])) {
          lastfoundindex = i;
          break;
        }
        else {
          lastfoundindex = i + 1;
        }
      }
      else {
        intersectFound = false;
        goalp[0] = (path[lastfoundindex][0]);
        goalp[1] = (path[lastfoundindex][1]);
      }
    }
    double abstargetangle = atan2(goalp[1] - currY, goalp[0] - currX)*180/pi;
    if (abstargetangle < 0) abstargetangle = abstargetangle + 360;
    double turnerror = abstargetangle - currTheta;
    if ((turnerror > 180) || (turnerror < -180)){
      turnerror = -1 * sign(turnerror) * (360 - abs(turnerror));
    }
    return {lastfoundindex, turnerror};
  }
}

void ppmove (double path[][2], int lastpointno) {
  MotorGroup bleft({-16, -20});
  MotorGroup bright({17, 19});
  bleft.setBrakeMode(AbstractMotor::brakeMode::coast);
  bright.setBrakeMode(AbstractMotor::brakeMode::coast);

  double lfi;
  double kp = 50;
  while (position.x != path[lastpointno][0] && position.y != path[lastpointno][1]) {
    auto [lastfoundindex, turnerror] = ppstep(path, lfi);
    lfi = lastfoundindex;
    double turnvel = kp*turnerror;
    bleft.moveVoltage(9000 + turnvel);
    bright.moveVoltage(9000 - turnvel);
    pros::delay(5);
  }
}
