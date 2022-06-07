#include "main.h"

void generate() {
#define ptlength 3
Waypoint points[ptlength];

//these points SHOULD run
Waypoint p1 = {-4, -1, d2r(45)};
Waypoint p2 = {-1, 2, 0};
Waypoint p3 = {2, 4, 0};
points[0] = p1;
points[1] = p2;
points[2] = p3;

TrajectoryCandidate candidate;
//adjust kinematic constraints
pathfinder_prepare(points, ptlength, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);
int length = candidate.length;

Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));
double wheelbasewidth = 0.6; //change
pathfinder_modify_tank(trajectory, length, trajectory, trajectory, wheelbasewidth);

int result = pathfinder_generate(&candidate, trajectory);
if (result < 0) {
  printf("Trajectory could not be generated. \n");
}

int i;
for (i = 0; i < length; i++) {
  Segment s = trajectory[i];
  printf("Time Step: %f\n", s.dt);
  printf("Coords: (%f, %f)\n", s.x, s.y);
  printf("Position (Distance): %f\n", s.position);
  printf("Velocity: %f\n", s.velocity);
  printf("Acceleration: %f\n", s.acceleration);
  printf("Jerk (Acceleration per Second): %f\n", s.jerk);
  printf("Heading (radians): %f\n", s.heading);
}

free(trajectory);

}
