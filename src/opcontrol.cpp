#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "odometry.hpp"
#include <iostream>
using namespace okapi;
using namespace std;

void opcontrol() {
  Controller masterController;
  MotorGroup bleft({-16, -20});
  MotorGroup bright({17, 19});
  bleft.setBrakeMode(AbstractMotor::brakeMode::coast);
  bright.setBrakeMode(AbstractMotor::brakeMode::coast);

  pros::Task Odometry(odom);

  while(true){
		//chassis
  double throttle = masterController.getAnalog(ControllerAnalog::rightY);
  double yaw = masterController.getAnalog(ControllerAnalog::leftX);
  double strafe = masterController.getAnalog(ControllerAnalog::rightX);
  double left = throttle + yaw;
  double right = throttle - yaw;
  bleft.moveVoltage(left*12000);
  bright.moveVoltage(right*12000);
  pros::delay(5);
  }
}
