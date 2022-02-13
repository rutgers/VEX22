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
	ks.kP = 0.002;
	ks.kI = 0;
	ks.kD = -0.000001;
	ks.kBias = 0;

	// Drive Motors
	front_rt1.reset(new okapi::Motor(11, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_rt2.reset(new okapi::Motor(6, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt1.reset(new okapi::Motor(14, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt2.reset(new okapi::Motor(1, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft1.reset(new okapi::Motor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft2.reset(new okapi::Motor(8, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_lft1.reset(new okapi::Motor(9, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_lft2.reset(new okapi::Motor(10, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));

	front_rt.reset(new okapi::MotorGroup({front_rt1, front_rt2}));
	front_lft.reset(new okapi::MotorGroup({front_lft1, front_lft2}));
	back_rt.reset(new okapi::MotorGroup({back_rt1, back_rt2}));
	back_lft.reset(new okapi::MotorGroup({back_lft1, back_lft2}));

	drive_lft.reset(new okapi::MotorGroup({front_lft1, front_lft2, back_lft1, back_lft2}));
	drive_rt.reset(new okapi::MotorGroup({front_rt1, front_rt2, back_rt1, back_rt2}));
	chassis = okapi::ChassisControllerBuilder()
		.withMotors(drive_lft, drive_rt)
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 14.5_in}, okapi::imev5GreenTPR})
		.withGains(ks, ks)
		.build();

	// Goal Catch Motors
	lift_front_lft.reset(new okapi::Motor(12, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front_rt.reset(new okapi::Motor(19, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front.reset(new okapi::MotorGroup({lift_front_lft, lift_front_rt}));
	lift_front->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	lift_front_control = okapi::AsyncPosControllerBuilder().withMotor(lift_front).build();

	lift_back_lft.reset(new okapi::Motor(13, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_back_rt.reset(new okapi::Motor(18, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_back.reset(new okapi::MotorGroup({lift_back_lft, lift_back_rt}));
	lift_back->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	lift_back_control = okapi::AsyncPosControllerBuilder().withMotor(lift_back).build();

	// Lift Front Motors
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
	chassis->setMaxVelocity(100.0);
	lift_front_control->setTarget((0.3)*3); 
	chassis->moveDistance(3.7_ft);
	chassis->waitUntilSettled();
	pros::delay(650);
	lift_front_control->setTarget((0.155)*3);
	pros::delay(650);
	chassis->moveDistance(-2_ft);
	chassis->waitUntilSettled();
	chassis->turnAngle(170_deg);
	chassis->waitUntilSettled();
	chassis->moveDistance(-1_ft);
	piston->set_value(true);
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
		double y = -master->get_analog(ANALOG_LEFT_Y);
		double x = 0;
		double z = master->get_analog(ANALOG_RIGHT_X);

		front_rt->moveVoltage((y+x+z)/127*11000);
		back_rt->moveVoltage((y-x+z)/127*11000);
    	front_lft->moveVoltage((y-x-z)/127*11000);
		back_lft->moveVoltage((y+x-z)/127*11000);

		if(master->get_digital(DIGITAL_A)) {
			lift_front_control->setTarget(FRONT_LIFT_DOWN);
		}
		else if(master->get_digital(DIGITAL_B)) {
			lift_front_control->setTarget(FRONT_LIFT_UP);
		} else if(lift_back->getPosition() < 0 && master->get_digital(DIGITAL_L2)){
			lift_front->moveVelocity(75);
		} else if( master->get_digital(DIGITAL_L1)) {
			lift_front->moveVelocity(-75);
		} else if(lift_front_control->isSettled()){
			lift_front->moveVelocity(0);
		}

		if(master->get_digital(DIGITAL_X)) {
			lift_back_control->setTarget(-(1.0/4.0)*BACK_LIFT_GEAR_RATIO);
		}
		else if(master->get_digital(DIGITAL_Y)) {
			lift_back_control->setTarget(0);
		} else if(master->get_digital(DIGITAL_R2)){
			lift_back->moveVelocity(-75);
		} else if(lift_back->getPosition() < 0 &&master->get_digital(DIGITAL_R1)) {
			lift_back->moveVelocity(75);
		} else if(lift_back_control->isSettled()){
			lift_back->moveVelocity(0);
		}

		if(piston_timer <= 0 && master->get_digital(DIGITAL_DOWN)) {
			piston_timer = 300;
			piston_flag = !piston_flag;
		}
		piston->set_value(piston_flag);
		if(piston_timer > 0) {
			piston_timer = piston_timer - 20;
		}
		pros::delay(20);
	}

}