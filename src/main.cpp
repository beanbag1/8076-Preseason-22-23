#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "odometry.hpp"
#include <iostream>
using namespace okapi;
using namespace std;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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

bool red = true;
int autonnum = 1;

void leftbtn() {
	autonnum--;
	if (autonnum < 1) autonnum = 4;
}

void centrebtn() {
	bool alreadychanged = false;
	if (red == true) {
		red = false;
		alreadychanged = true;
	}
	else if (red == false && alreadychanged == false) {
		red = true;
		alreadychanged = true;
	}
}

void rightbtn() {
	autonnum++;
	if (autonnum > 4) autonnum = 1;
}

void competition_initialize() {
	while (true) {
		pros::lcd::register_btn0_cb(leftbtn);
		pros::lcd::register_btn1_cb(centrebtn);
		pros::lcd::register_btn2_cb(rightbtn);

    if(red == true) {
			pros::lcd::set_text(0, "Red.");
		} else if (red == false) {
			pros::lcd::set_text(0, "Blue.");
		}
		pros::lcd::print(1, "Auton selected: %d", autonnum);
		pros::delay(20);
  }
}
