#include "SKAR_2.hpp"

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

int FRONT_LIFT_GEAR_RATIO = 7;
int BACK_LIFT_GEAR_RATIO = 7;

double FRONT_LIFT_DOWN = -(1.5/8.0)*FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_UP = -(0.5/8.0)*FRONT_LIFT_GEAR_RATIO;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

	// PID Variables
	ks.kP = 0.0016;
	ks.kI = 0;
	ks.kD = -0.000001;
	ks.kBias = 0;

	// Drive Motors
	frontFrontLft.reset(new okapi::Motor(15, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	frontLft.reset(new okapi::Motor(3, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	backLft.reset(new okapi::Motor(2, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	backBackLft.reset(new okapi::Motor(1, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));

	frontFrontRt.reset(new okapi::Motor(16, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	frontRt.reset(new okapi::Motor(8, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	backRt.reset(new okapi::Motor(9, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	backBackRt.reset(new okapi::Motor(10, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));

	drive_lft.reset(new okapi::MotorGroup({frontFrontLft, frontLft, backLft, backBackLft}));
	drive_rt.reset(new okapi::MotorGroup({frontFrontRt, frontRt, backRt, backBackRt}));
	chassis = okapi::ChassisControllerBuilder()
		.withMotors(drive_lft, drive_rt)
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 14.5_in}, okapi::imev5GreenTPR})
		.withGains(ks, ks)
		.build();

	// Back Lift
	backLeftLift.reset(new okapi::Motor(11, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	backRtLift.reset(new okapi::Motor(20, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	back_lift.reset(new okapi::MotorGroup({backLeftLift, backRtLift}));
	back_lift->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	// Front Lift
	frontLftLift.reset(new okapi::Motor(18, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	frontRtLift.reset(new okapi::Motor(13, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	front_lift.reset(new okapi::MotorGroup({frontLftLift, frontRtLift}));
	front_lift->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	// Controller Initialization
	master.reset(new pros::Controller(pros::E_CONTROLLER_MASTER));

	// Pneumatics
	piston.reset(new pros::ADIDigitalOut('A'));
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

	// goalratio 1:3
	//chassis->setMaxVelocity(100.0);
	//lift_front_control->setTarget(FRONT_LIFT_DOWN); 
	chassis->moveDistance(-6.9_ft);
	pros::delay(300);
	pros::delay(300);
	chassis->moveDistance(5.5_ft);
	// chassis->turnAngle(185_deg);
	// lift_front_control->setTarget(FRONT_LIFT_DOWN);
	// piston->set_value(false);
	
	// lift_back_control->setTarget(-(1.2/32.0)*BACK_LIFT_GEAR_RATIO);
	// chassis->moveDistance(6_ft);
	// piston->set_value(true);
	// chassis->moveDistance(-6_ft);

	//Drop ring;
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
	int piston_timer = 0;
	bool piston_flag = false;
	while (true){

		// Drive Mechanics
		double y_l = master->get_analog(ANALOG_LEFT_Y);
		double y_r = master->get_analog(ANALOG_RIGHT_Y);
		drive_lft->moveVelocity(y_l);
		drive_rt->moveVelocity(y_r);

		// Back Lifting Mechanics 
		// Note: Make speed a little slower
		if(master->get_digital(DIGITAL_R1)){ // Up
			back_lift->moveVelocity(-50);
		} else if(master->get_digital(DIGITAL_R2)){ // Down
			back_lift->moveVelocity(50);
		} else {
			back_lift->moveVelocity(0);
		}

		// Front Lifting Mechanics
		if(master->get_digital(DIGITAL_L1)) { // Up
			front_lift->moveVelocity(50);
		} else if(master->get_digital(DIGITAL_L2)){ // Down
 			front_lift->moveVelocity(-50);
		} else {
			front_lift->moveVelocity(0);
		}

		pros::delay(20);
	}

}