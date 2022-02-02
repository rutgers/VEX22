#include "main.h"
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
	okapi::Motor front_rt(14, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_rt(5, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor front_lft(15, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_lft(4, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup drive_lft({front_lft, back_lft});
	okapi::MotorGroup drive_rt({front_rt, back_rt});

	okapi::IterativePosPIDController::Gains ks; 
	ks.kP = 0.002;
	ks.kI = 0;
	ks.kD = -0.000001;
	ks.kBias = 0;

	std::shared_ptr<okapi::ChassisController> chassis =
	okapi::ChassisControllerBuilder()
		.withMotors(drive_lft, drive_rt)
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 11.5_in}, okapi::imev5GreenTPR})
		.withGains(ks, ks)
		.build();
	
	okapi::Motor goalcatch_lft(9, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor goalcatch_rt(10, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup goalcatch({goalcatch_lft, goalcatch_rt});
	goalcatch.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	std::shared_ptr<okapi::AsyncPositionController<double, double>> goalcatch_control =
  		okapi::AsyncPosControllerBuilder().withMotor(goalcatch).build();

	okapi::Motor lift_lft(20, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor lift_rt(11, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup lift({lift_lft, lift_rt});
	lift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	std::shared_ptr<okapi::AsyncPositionController<double, double>> lift_control =
  		okapi::AsyncPosControllerBuilder().withMotor(lift).build();

	okapi::Motor intake(16, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	std::shared_ptr<okapi::AsyncMotionProfileController> profileController =
	okapi::AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			2.0, // Maximum linear acceleration of the Chassis in m/s/s
			10.0 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(chassis)
		.buildMotionProfileController();
	
	profileController->generatePath({
		{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
		{10_ft, 0_ft, 0_deg}, // The next point in the profile, 3 feet forward
		{10_ft, 0_ft, 90_deg}}, // The next point in the profile, 3 feet forward
		"A" // Profile name
	);

// 	profileController->setTarget("A");
// 	profileController->waitUntilSettled();
	lift_control->setTarget((2.9/8.0)*7);
	chassis->moveDistance(11_ft);
	chassis->waitUntilSettled();
	lift_control->waitUntilSettled();
	lift_control->setTarget(1.0/8.0*7);
	pros::delay(1000);
	lift_control->waitUntilSettled();
	chassis->moveDistance(-12_ft);
	chassis->waitUntilSettled();
	chassis->moveDistance(2_ft);
	chassis->waitUntilSettled();
	chassis->turnAngle(-250_deg);
	chassis->waitUntilSettled();
	chassis->moveDistance(-1_ft);
	chassis->waitUntilSettled();
	lift_control->setTarget(1.5/8.0*7);
	intake.moveVoltage(-4500);
	lift_control->setTarget((2.9/8.0)*7);
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
	int goalCatchPower = 100;

	// Driving Motors
	okapi::Motor front_rt(14, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_rt(5, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor front_lft(15, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_lft(4, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup drive_lft({front_lft, back_lft});
	okapi::MotorGroup drive_rt({front_rt, back_rt});
	std::shared_ptr<okapi::ChassisController> chassis =
	okapi::ChassisControllerBuilder()
		.withMotors({-15, -4}, {14, 5})
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 11.5_in}, okapi::imev5GreenTPR})
		.build();

	// Goalcatch Motors
	okapi::Motor goalcatch_lft(9, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor goalcatch_rt(10, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup goalcatch({goalcatch_lft, goalcatch_rt});
	goalcatch.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	okapi::Motor goalcatch_lft2(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor goalcatch_rt2(11, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup goalcatch2({goalcatch_lft2, goalcatch_rt2});
	goalcatch2.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	// Intake Motor
	okapi::Motor intake(16, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	int intake_flag = 0;

	while (true){

		// Driving Mechanics
		double y = -master.get_analog(ANALOG_LEFT_Y);
		double x = -master.get_analog(ANALOG_LEFT_X);
		double z = -master.get_analog(ANALOG_RIGHT_X);

		front_rt.moveVoltage((-y+x+z)/127*11000);
		back_rt.moveVoltage((y+x-z)/127*11000);
    	front_lft.moveVoltage((-y-x-z)/127*11000);
		back_lft.moveVoltage((y-x+z)/127*11000);

		// Goal Catch Mechanics
		if(master.get_digital(DIGITAL_R1)){
			goalcatch.moveVelocity(goalCatchPower*100);
		} else if(master.get_digital(DIGITAL_R2)) {
			goalcatch.moveVelocity(-goalCatchPower*100);
		} else {
			goalcatch.moveVelocity(0);
		}

		if(master.get_digital(DIGITAL_L1)){
			goalcatch2.moveVelocity(-goalCatchPower*100);
		} else if(master.get_digital(DIGITAL_L2)) {
			goalcatch2.moveVelocity(+goalCatchPower*100);
		} else {
			goalcatch2.moveVelocity(0);
		}

		// Intake
		if(master.get_digital(DIGITAL_UP)) {
			intake_flag = 1;
		}
		else if(master.get_digital(DIGITAL_DOWN)) {
			intake_flag = -1;
		}
		else if(master.get_digital(DIGITAL_RIGHT) || master.get_digital(DIGITAL_LEFT)) {
			intake_flag = 0;
		}
		if(intake_flag == 1) {
			intake.moveVoltage(-9000);
		} else if(intake_flag == -1){
			intake.moveVoltage(9000);
		}else {
			intake.moveVoltage(0);
		}

		pros::delay(20);
	}
}