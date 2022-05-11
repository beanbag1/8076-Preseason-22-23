#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "odometry.hpp"
#include <iostream>
using namespace okapi;
using namespace std;

//global control variables
//GPS
double xerror;
double yerror;
double heading1;
//pistons
bool fpbin;
bool bpbin;
bool brpbin;
bool tiltbin;
bool busy;
//status controller
double brstatus;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Controller masterController;
	Controller partnerController(ControllerId::partner);
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
	Motor branch(1);
	Motor bl(2);
	Motor fl(-3);
	Motor intake(-4);
	auto trackerL = ADIEncoder('C', 'D', true);
	auto trackerR = ADIEncoder('A', 'B', true);
	pros::ADIDigitalOut frontpiston ('E');
	pros::ADIDigitalOut backpiston ('F');
	pros::ADIDigitalOut branchpiston ('G');
	pros::ADIDigitalOut tiltpiston ('H');
	// pros::Gps GPS(11, -0.00635, 0.127);
	pros::Vision visionfront(5, pros::E_VISION_ZERO_CENTER);
	pros::Vision visionback(6, pros::E_VISION_ZERO_CENTER);
	//reset encoders
	trackerR.reset();
	trackerL.reset();
	//reset lifts
	fl.tarePosition();
	bl.tarePosition();
	branch.tarePosition();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
