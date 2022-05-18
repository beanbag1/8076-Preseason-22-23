#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "odometry.hpp"
#include "auton.hpp"
#include <iostream>
using namespace okapi;
using namespace std;

void autonomous() {
  pros::Task Odometry(odom);
  double path[][2] = {{-24, -24}, {-24, 24}, {24, 24}, {24,-24}, {-24, -24}};
  ppmove(path, 5);
}
