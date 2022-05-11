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
  ControllerButton brlow(ControllerId::partner, ControllerDigital::B);
  ControllerButton brmid(ControllerId::partner, ControllerDigital::A);
  ControllerButton brup(ControllerId::partner, ControllerDigital::X);
  ControllerButton fpbut(ControllerId::master, ControllerDigital::L1);
  ControllerButton bpbut(ControllerId::master, ControllerDigital::R1);
  ControllerButton brpbut(ControllerId::partner, ControllerDigital::left);
  ControllerButton brpbut1(ControllerId::partner, ControllerDigital::right);
  ControllerButton brpbut2(ControllerId::partner, ControllerDigital::up);
  ControllerButton brpbut3(ControllerId::partner, ControllerDigital::down);
  ControllerButton tiltbut(ControllerId::master, ControllerDigital::R2);
  MotorGroup bleft({-16, -20});
  MotorGroup bright({17, 19});
  MotorGroup drive({17, -16, 19, -20});
  drive.setBrakeMode(AbstractMotor::brakeMode::brake);
	bleft.setBrakeMode(AbstractMotor::brakeMode::brake);
	bright.setBrakeMode(AbstractMotor::brakeMode::brake);

  std::shared_ptr<ChassisController> driveController =
  ChassisControllerBuilder()
  .withMotors(bleft, bright)
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
  .build();

  pros::Task Odometry(odom);

  while(true){
		//chassis
		driveController -> getModel() -> arcade(masterController.getAnalog(ControllerAnalog::rightY), masterController.getAnalog(ControllerAnalog::leftX));
    pros::delay(5);
  }
}
