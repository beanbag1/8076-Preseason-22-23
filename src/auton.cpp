#include "main.h"

void autonomous() {
  pros::Task Odometry(odom);
  double path[][2] = {{-24, -24}, {-24, 24}, {24, 24}, {24,-24}, {-24, -24}};
  ppmove(path, 5);
}
