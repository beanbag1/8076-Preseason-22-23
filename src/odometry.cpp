//everything is in inches unless otherwise stated

#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include <iostream>
using namespace okapi;
using namespace std;

//declare base stuff
auto trackerL = ADIEncoder('C', 'D', true);
auto trackerR = ADIEncoder('A', 'B', true);
MotorGroup bleft({-16, -20});
MotorGroup bright({17, 19});
MotorGroup drive({17, -16, 19, -20});

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
    position.theta = (lastTheta + (currLeft - currRight)*inchesperdeg/sTotal)*180/pi; //converts radians to degrees
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

    //printing to brain screen
    pros::screen::print(TEXT_LARGE, 1, "X-coordinate: ", position.x);
    pros::screen::print(TEXT_LARGE, 2, "Y-coordinate: ", position.y);
    pros::screen::print(TEXT_LARGE, 3, "Theta: ", position.theta);

    pros::Task::delay(5); //wait 5ms before running again
  }
}
