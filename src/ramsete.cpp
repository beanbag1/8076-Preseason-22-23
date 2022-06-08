#include "main.h"

#define beta 1
#define zeta 0.5
//i need to tune these TT

void ramseteFollow(double x, double y, double theta, double leftvel, double rightvel) {
  double globalxError = x - position.x;
  double globalyError = y - position.y;
  double globalthetaError = theta - position.theta;
  //stupid matrices shit
  double localxError = (cos(position.theta)*globalxError + sin(position.theta)*globalyError);
  double localyError = (-sin(position.theta)*globalxError + cos(position.theta)*globalyError);
  double localthetaError = globalthetaError;
  //find angular and linear vels from leftnright vels
  double desiredangularvel = ((leftvel - rightvel)/2);
  double desiredlinearvel = leftvel - desiredangularvel;
  double k = 2 * zeta * sqrt(pow(desiredangularvel, 2) + beta * pow(desiredlinearvel, 2));

  double adjustedlinearvel = desiredlinearvel*cos(localthetaError) + k*localxError;
  double adjustedangularvel = desiredangularvel + k*localthetaError + ((beta*desiredlinearvel*sin(localthetaError)*localyError)/localthetaError);

  double linearmotorvel = adjustedlinearvel/pi/4;
  double angularmotorvel = adjustedangularvel/pi/4;

  double baseconstant = 1; //need to find what this is
  //then movevoltage the motorvels multiplied by the constant
}
