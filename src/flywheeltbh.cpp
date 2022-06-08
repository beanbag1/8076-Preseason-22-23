#include "main.h"

double desiredvel = 0;

int sign(double num) {
  if (num >= 0) return 1;
  else return -1;
}

void flywheelTBHcontrol() {
  Motor flywheel(1);
  double TBHInitial = 1;
  double TBH = TBHInitial;
  double actualvel = flywheel.getActualVelocity();
  double error = desiredvel - actualvel;
  double integral = 0;
  double errorsign = 1;
  double correctedvel = 0;
  while(true) {
    error = 200 - actualvel;
    integral = integral + error;
    if(sign(error) == errorsign) {
      correctedvel = integral*TBH;
    } else if (sign(error) != errorsign) {
      TBH = TBH/2;
      correctedvel = integral*TBH;
      if (errorsign == 1) errorsign = -1;
      else if (errorsign == -1) errorsign = 1;
    }
    flywheel.moveVelocity(correctedvel);
    pros::Task::delay(5);
  }
}
