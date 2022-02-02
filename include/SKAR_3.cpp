#include "main.h"
//#include "DriveTrain.cpp"
#include "okapi/api.hpp"
#include <vector>
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {
	
 }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	
	// Controller
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// Drive Motors

	// Front Right Motors
	okapi::Motor front_rt_1(13, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor front_rt_2(15, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	
	// Front Left Motors
	okapi::Motor front_lft_1(18, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor front_lft_2(16, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	
	
	// Back Right Motors
	okapi::Motor back_rt_1(3, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_rt_2(5, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	
	// Back Left Motors
	okapi::Motor back_lft_1(6, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_lft_2(8, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations);
	
	// Group Motors
	okapi::MotorGroup front_rt({front_rt_1, front_rt_2});
	okapi::MotorGroup front_lft({front_lft_1, front_lft_2});
	okapi::MotorGroup back_rt({back_rt_1, back_rt_2});
	okapi::MotorGroup back_lft({back_lft_1, back_lft_2});

	// Chasis Motors Grouped
	/* okapi::ChassisControllerBuilder()
		.withMotors(drive_lft, drive_rt)
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::blue, {{4_in, 11.5_in}, okapi::imev5BlueTPR})
		.build(); */

	// Claw Motors
	okapi::Motor clawFL(20, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor clawFR(11, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor clawBL(1, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor clawBR(10, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup clawR({clawFR, clawBR});
	clawR.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	okapi::MotorGroup clawL({clawFL, clawBL});
	clawL.setBrakeMode(okapi::AbstractMotozr::brakeMode::hold);

	while (true){
		// Drive Mechanics
		double y = master.get_analog(ANALOG_LEFT_Y);
		double x = master.get_analog(ANALOG_LEFT_X);
		double z = master.get_analog(ANALOG_RIGHT_X);
		front_rt.moveVelocity(y+x+z);
		back_rt.moveVelocity(y-x+z);
    	front_lft.moveVelocity(y-x-z);
		back_lft.moveVelocity(y+x-z);

		if(master.get_digital(DIGITAL_A)) {
			clawL.moveVelocity(100);
		} else if(master.get_digital(DIGITAL_B)){
			clawL.moveVelocity(-100);
		} else {
			clawL.moveVelocity(0);
		}

		if(master.get_digital(DIGITAL_X)) {
			clawR.moveVelocity(100);
		} else if(master.get_digital(DIGITAL_Y)){
			clawR.moveVelocity(-100);
		} else {
			clawR.moveVelocity(0);
		}

		pros::delay(20);
	}
/*
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor front_rt(14);
	pros::Motor back_rt(1);
	pros::Motor front_lft(15);
	pros::Motor back_lft(2);

	//9 and 10 goalcatch motors
	pros::Motor goalcatch_lft(9);
	pros::Motor goalcatch_rt(10);

	goalcatch_rt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	goalcatch_lft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// 16 intake motor
	pros::Motor intake(16);

	while (true){

		// Driving Mechanics
		double y = -master.get_analog(ANALOG_LEFT_X);
		double x = -master.get_analog(ANALOG_LEFT_Y);
		double z = -master.get_analog(ANALOG_RIGHT_X);

		front_rt.move(y-x-z);
		back_rt.move(y+x-z);
    	front_lft.move(y+x+z);
		back_lft.move(y-x+z);

		// Goal Catch Mechanics
		if(master.get_digital(DIGITAL_A)){
			goalcatch_lft.move(-goalCatchPower);
			goalcatch_rt.move(goalCatchPower);
		} else if(master.get_digital(DIGITAL_B)) {
			goalcatch_rt.move(-goalCatchPower);
			goalcatch_lft.move(goalCatchPower);
		} else {
			goalcatch_rt.move(0);
			goalcatch_lft.move(0);
		}

		// Intake
		if(master.get_digital(DIGITAL_L2)) {
			intake.move(-90);
		} else if(master.get_digital(DIGITAL_R2)){
			intake.move(90);
		}else {
			intake.move(0);
		}

		pros::delay(20);
	}*/
}