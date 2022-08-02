//everything is in inches unless otherwise stated
#include "main.h"

Coordinates position;

//declare base stuff
auto trackerC = ADIEncoder('C', 'D', true);
auto trackerS = ADIEncoder('A', 'B', true);
pros::IMU inertial(1);

//define FIXED values
#define sC 0
#define sS 5
#define inchesperdeg (3.25*pi/360)

//define CHANGING values
double currC = 0;
double currS = 0;
double bearing = 0;

void odom() {
  double prevEncdC = 0;
  double prevEncdS = 0;
  double prevBearing = 0;
  int count = 0;
  while(true) {
    currC = (trackerC.get()*inchesperdeg);
    currS = (trackerS.get()*inchesperdeg);
    bearing = (inertial.get_heading()/180*pi);
    double encdChangeC = currC-prevEncdC;
    double encdChangeS = currS-prevEncdS;
    double bearingChange = bearing - prevBearing;
    //set prev variables
    prevEncdC = currC;
    prevEncdS = currS;
    prevBearing = bearing;
    double localXoffset = 0;
    double localYoffset = 0;
    if (bearingChange) {
      localXoffset = (encdChangeS/bearingChange + sS)*2*sin(bearingChange/2);
      localYoffset = (encdChangeC/bearingChange + sC)*2*sin(bearingChange/2);
    } else {
      localXoffset = encdChangeS;
      localYoffset = encdChangeC;
    }

    double avgBearing = prevBearing + bearingChange/2;
    double rotatedXoffset = 0;
    double rotatedYoffset = 0;
    rotatedXoffset = cos(-avgBearing)*localXoffset - sin(-avgBearing)*localYoffset;
    rotatedYoffset = sin(-avgBearing)*localXoffset + cos(-avgBearing)*localYoffset;
    position.x = position.x + rotatedXoffset;
    position.y = position.y + rotatedYoffset;
    position.theta = inertial.get_heading();
    pros::Task::delay(5); //wait 5ms before running again
  }
}

// double sign(double num){
//   if (num >= 0) return 1;
//   else return -1;
// }

// double ptpdist(double p1[], double p2[]){
//   double dist = sqrt((p2[0] - p1[0])*(p2[0] - p1[0]) + (p2[1] - p1[1])*(p2[1] - p1[1]));
//   return dist;
// }

// tuple<double, double> ppstep(double path[][2], double LFindex) {
//   bool intersectFound = false;
//   double currX = position.x;
//   double currY = position.y;
//   double currTheta = position.theta;
//   double goalp[] = {};
//   double lookaheaddist = 0.001;
//   int lastfoundindex = LFindex;
//   int startingindex = lastfoundindex;
//   double currentpos[] = {currX, currY};
//   int i;
//   for(i = startingindex; i < (sizeof(path)/sizeof(path[0])-1);) {
//     double x1 = path[i][0] - currX;
//     double y1 = path[i][1] - currY;
//     double x2 = path[i+1][0] - currX;
//     double y2 = path[i+1][1] - currY;
//     double dx = x2 - x1;
//     double dy = y2 - y1;
//     double dr = sqrt(dx*dx + dy*dy);
//     double d = x1*y2 - x2*y1;
//     double discriminant = (lookaheaddist*lookaheaddist)*(dr*dr) - d*d;
//     if (discriminant >= 0) {
//       double solx1 = (d*dy + sign(dy)*dx*sqrt(discriminant))/(dr*dr);
//       double solx2 = (d*dy - sign(dy)*dx*sqrt(discriminant))/(dr*dr);
//       double soly1 = (-d*dx + abs(dy)*sqrt(discriminant))/(dr*dr);
//       double soly2 = (-d*dx - abs(dy)*sqrt(discriminant))/(dr*dr);
//       double solp1[] = {(solx1 + currX), (soly1 + currY)};
//       double solp2[] = {(solx2 + currX), (soly2 + currY)};
//       double minX = min(path[i][0], path[i+1][0]);
//       double maxX = max(path[i][0], path[i+1][0]);
//       double minY = min(path[i][1], path[i+1][1]);
//       double maxY = max(path[i][1], path[i+1][1]);
//       if (((minX <= solp1[0] <= maxX) && (minY <= solp1[1] <= maxY)) || ((minX <= solp2[0] <= maxX) && (minY <= solp2[1] <= maxY))) {
//         intersectFound = true;
//         if (((minX <= solp1[0] <= maxX) && (minY <= solp1[1] <= maxY)) && ((minX <= solp2[0] <= maxX) && (minY <= solp2[1] <= maxY))) {
//           if (ptpdist(solp1, path[i+1]) < ptpdist(solp2, path[i+1])) {
//             goalp[0] = solp1[0];
//             goalp[1] = solp1[1];
//           }
//           else {
//             goalp[0] = solp2[0];
//             goalp[1] = solp2[1];
//           }
//         }
//         else {
//           if ((minX <= solp1[0] <= maxX) && (minY <= solp1[1] <= maxY)) {
//             goalp[0] = solp1[0];
//             goalp[1] = solp1[1];
//           }
//           else {
//             goalp[0] = solp2[0];
//             goalp[1] = solp2[1];
//           }
//         }
//         if (ptpdist(goalp, path[i+1]) < ptpdist(currentpos, path[i+1])) {
//           lastfoundindex = i;
//           break;
//         }
//         else {
//           lastfoundindex = i + 1;
//         }
//       }
//       else {
//         intersectFound = false;
//         goalp[0] = (path[lastfoundindex][0]);
//         goalp[1] = (path[lastfoundindex][1]);
//       }
//     }
//     double abstargetangle = atan2(goalp[1] - currY, goalp[0] - currX)*180/pi;
//     if (abstargetangle < 0) abstargetangle = abstargetangle + 360;
//     double turnerror = abstargetangle - currTheta;
//     if ((turnerror > 180) || (turnerror < -180)){
//       turnerror = -1 * sign(turnerror) * (360 - abs(turnerror));
//     }
//     return {lastfoundindex, turnerror};
//   }
// }

// void ppmove (double path[][2], int lastpointno) {
//   MotorGroup bleft({-16, -20});
//   MotorGroup bright({17, 19});
//   bleft.setBrakeMode(AbstractMotor::brakeMode::coast);
//   bright.setBrakeMode(AbstractMotor::brakeMode::coast);

//   double lfi;
//   double kp = 50;
//   while (position.x != path[lastpointno][0] && position.y != path[lastpointno][1]) {
//     auto [lastfoundindex, turnerror] = ppstep(path, lfi);
//     lfi = lastfoundindex;
//     double turnvel = kp*turnerror;
//     bleft.moveVoltage(9000 + turnvel);
//     bright.moveVoltage(9000 - turnvel);
//     pros::delay(5);
//   }
// }
