#include "main.h"

void opcontrol() {
  Controller masterController;
  Motor leftF (-1);
  Motor leftB (-2);
  Motor rightF (9);
  Motor rightB (10);
  MotorGroup base ({-1, -2, 9, 10});
  base.setBrakeMode(AbstractMotor::brakeMode::coast);
  while(true){
		//chassis
  double throttle = masterController.getAnalog(ControllerAnalog::rightY);
  double yaw = masterController.getAnalog(ControllerAnalog::leftX);
  double strafe = masterController.getAnalog(ControllerAnalog::rightX);
  leftF.moveVelocity((throttle + yaw + strafe)*200);
  leftB.moveVelocity((throttle + yaw - strafe)*200);
  rightF.moveVelocity((throttle - yaw - strafe)*200);
  rightB.moveVelocity((throttle - yaw + strafe)*200);
  pros::delay(5);
  }
}
