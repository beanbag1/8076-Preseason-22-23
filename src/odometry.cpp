//everything is in inches unless otherwise stated

#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "odometry.hpp"
#include <iostream>
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
    if (position.theta > 2*pi) {
      position.theta = position.theta - 2*pi;
    } else if (position.theta < 2*pi) {
      position.theta = position.theta + 2*pi;
    } else break;
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
    pros::screen::print(TEXT_LARGE, 1, "X-coordinate: %f", position.x);
    pros::screen::print(TEXT_LARGE, 3, "Y-coordinate: %f", position.y);
    pros::screen::print(TEXT_LARGE, 5, "Theta: %f", position.theta*180/pi);
    std::cout << position.x << " " << position.y << " " << position.theta << std::endl;

    pros::Task::delay(5); //wait 5ms before running again
  }
}
