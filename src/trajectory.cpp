#include "main.h"

void generate() {
//inchespersec etc.
#define maxVel (280/60)*(4*pi)
#define maxAccel 1
#define maxJerk 1

#define ptlength 3
Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * ptlength);;

//these points SHOULD run
Waypoint p1 = {-4, -1, d2r(45)};
Waypoint p2 = {-1, 2, 0};
Waypoint p3 = {2, 4, 0};
points[0] = p1;
points[1] = p2;
points[2] = p3;

TrajectoryCandidate candidate;
//adjust kinematic constraints
pathfinder_prepare(points, ptlength, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.005, maxVel, maxAccel, maxJerk, &candidate);
free(points);

int length = candidate.length;
Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));

pathfinder_generate(&candidate, trajectory);

double wheelbasewidth = 0.6; //change to inches something
Segment *lefttrajectory = (Segment*)malloc(length * sizeof(Segment));
Segment *righttrajectory = (Segment*)malloc(length * sizeof(Segment));
pathfinder_modify_tank(trajectory, length, lefttrajectory, righttrajectory, wheelbasewidth);

int i;
for (i = 0; i < length; i++) {
  Segment leftS = lefttrajectory[i];
  Segment rightS = righttrajectory[i];
  ramseteFollow(leftS.x, leftS.y, leftS.heading, leftS.velocity, rightS.velocity);
  pros::delay(5); //the number here should be equal to your looptime declared
}

free(trajectory);
free(lefttrajectory);
free(righttrajectory);

}
