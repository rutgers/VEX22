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
//goalcatch gear ratio 1:5
//lift gear ratio 1:7
double goalcatch_down = 1.25;


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

	okapi::IterativePosPIDController::Gains ks; 
	ks.kP = 0.002;
	ks.kI = 0;
	ks.kD = -0.000001;
	ks.kBias = 0;

	// Drive Motors
	okapi::Motor front_rt(6, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_rt(7, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor front_lft(16, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_lft(19, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup drive_lft({front_lft, back_lft});
	okapi::MotorGroup drive_rt({front_rt, back_rt});
	std::shared_ptr<okapi::ChassisController> chassis =
	okapi::ChassisControllerBuilder()
		.withMotors(drive_lft, drive_rt)
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 14.5_in}, okapi::imev5GreenTPR})
		.withGains(ks, ks)
		.build();

	// Goal Catch Motors
	okapi::Motor goalcatch_lft(15, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor goalcatch_rt(8, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup goalcatch({goalcatch_lft, goalcatch_rt});
	goalcatch.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	std::shared_ptr<okapi::AsyncPositionController<double, double>> goalcatch_control =
  		okapi::AsyncPosControllerBuilder().withMotor(goalcatch).build();

	// Set Profile Controller for Main Autonmous Functions
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
		{10_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
		//{10_ft, 0_ft, 90_deg}}, // The next point in the profile, 3 feet forward
		"A" // Profile name
	);

	// goalratio 1:3
	chassis->setMaxVelocity(100.0);
	goalcatch_control->setTarget((0.3)*3); 
	chassis->moveDistance(-3.7_ft);
	chassis->waitUntilSettled();
	pros::delay(650);
	goalcatch_control->setTarget((0.155)*3);
	pros::delay(650);
	chassis->moveDistance(2_ft);
	chassis->waitUntilSettled();
	//chassis->turnAngle(170_deg);
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
	okapi::Motor front_rt(6, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_rt(7, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor front_lft(16, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor back_lft(17, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup drive_lft({front_lft, back_lft});
	okapi::MotorGroup drive_rt({front_rt, back_rt});
	okapi::ChassisControllerBuilder()
		.withMotors(drive_lft, drive_rt)
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 11.5_in}, okapi::imev5GreenTPR})
		.build();

	// Front Lift Motors
	okapi::Motor frontRLft(5, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor frontLLft(15, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations);

	// Lift Motors
	/* okapi::Motor liftTL(1, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor liftBL(9, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor liftTR(2, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor liftBR(10, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	okapi::MotorGroup liftL({liftTL, liftBL});
	liftL.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	okapi::MotorGroup liftR({liftTR, liftBR});
	liftR.setBrakeMode(okapi::AbstractMotor::brakeMode::hold); */

	// Claw Motors
	okapi::Motor claw(11, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
	claw.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	while (true){

		// Drive Mechanics
		double y = -master.get_analog(ANALOG_LEFT_Y);
		double x = master.get_analog(ANALOG_LEFT_X);
		double z = master.get_analog(ANALOG_RIGHT_X);
		front_rt.moveVelocity((y+x+z)*100);
		back_rt.moveVelocity((y-x+z)*100);
    	front_lft.moveVelocity((y-x-z)*100);
		back_lft.moveVelocity((y+x-z)*100);

		// Goal Catch Mechanics 
		if(master.get_digital(DIGITAL_A)){
			frontRLft.moveVelocity(1200);
			frontLLft.moveVelocity(-1200);
		} else if(master.get_digital(DIGITAL_B)) {
			frontRLft.moveVelocity(-1200);
			frontLLft.moveVelocity(1200);
		} else {
			frontRLft.moveVelocity(0);
			frontLLft.moveVelocity(0);
		}

		// Lift Mechanics
		if(master.get_digital(DIGITAL_L1)) {
			liftL.moveVelocity(50);
			liftR.moveVelocity(-50);
		} else if(master.get_digital(DIGITAL_R1)) {
			liftL.moveVelocity(-50);
			liftR.moveVelocity(50);
		} else {
			liftL.moveVelocity(0);
			liftR.moveVelocity(0);
		}

		// Claw Mechanics
		if(master.get_digital(DIGITAL_R2)) {
			claw.moveVelocity(10);
		} else if(master.get_digital(DIGITAL_L2)) {
			claw.moveVelocity(-10);
		} else {
			claw.moveVelocity(0);
		}

		pros::delay(20);
	}

}