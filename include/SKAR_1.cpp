#include "SKAR_1.hpp"

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

	front_rt.reset(new okapi::Motor(5, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt.reset(new okapi::Motor(2, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft.reset(new okapi::Motor(6, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_lft.reset(new okapi::Motor(15, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	drive_lft.reset(new okapi::MotorGroup({*front_lft, *back_lft}));
	drive_rt.reset(new okapi::MotorGroup({*front_rt, *back_rt}));
 
	ks.kP = 0.0010;
	ks.kI = 0;
	ks.kD = -0.000002;
	ks.kBias = 0;

	chassis = okapi::ChassisControllerBuilder()
		.withMotors(drive_lft, drive_rt)
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 11.5_in}, okapi::imev5GreenTPR})
		.withGains(ks, ks)
		.build();
	
	lift_front_lft.reset(new okapi::Motor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front_rt.reset(new okapi::Motor(11, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front.reset(new okapi::MotorGroup({*lift_front_lft, *lift_front_rt}));
	lift_front->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	lift_front_control = okapi::AsyncPosControllerBuilder().withMotor(*lift_front).build();

	lift_back_lft.reset(new okapi::Motor(10, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	lift_back_rt.reset(new okapi::Motor(1, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	lift_back.reset(new okapi::MotorGroup({lift_back_lft, lift_back_rt}));
	lift_back->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	lift_back_control =	okapi::AsyncPosControllerBuilder().withMotor(lift_back).build();

	intake.reset(new okapi::Motor(17, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	master.reset(new pros::Controller(pros::E_CONTROLLER_MASTER));
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
	//Lift Gear ratio 1:5
	lift_front_control->setTarget((3.0/8.0)*LIFT_GEAR_RATIO);
	chassis->moveDistance(6_ft);
	chassis->waitUntilSettled();
	lift_front_control->waitUntilSettled();
	lift_front_control->setTarget(2/8.0*LIFT_GEAR_RATIO);
	pros::delay(1000);
	lift_front_control->waitUntilSettled();
	chassis->moveDistance(-5.6_ft);
	chassis->waitUntilSettled();
	chassis->moveDistance(1_ft);
	chassis->waitUntilSettled();
	chassis->turnAngle(-140_deg);
	chassis->waitUntilSettled();
	lift_back_control->setTarget(-3.0/8.0*LIFT_GEAR_RATIO);
	lift_back_control->waitUntilSettled();
	chassis->moveDistance(-3_ft);
	chassis->waitUntilSettled();
	lift_back_control->setTarget(-(2/8.0)*LIFT_GEAR_RATIO);
	chassis->moveDistance(3_ft);
	intake->moveVoltage(12000);
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
	int intake_flag = 0;
	int delay = 0;
	bool chassis_hold = false;

	while (true){

		// Driving Mechanics
		double y = master->get_analog(ANALOG_LEFT_Y);
		double x = -master->get_analog(ANALOG_LEFT_X);
		double z = -master->get_analog(ANALOG_RIGHT_X);

		front_rt->moveVoltage((y+x+z)/127*11000);
		back_rt->moveVoltage((y-x+z)/127*11000);
    	front_lft->moveVoltage((y-x-z)/127*11000);
		back_lft->moveVoltage((y+x-z)/127*11000);

		if(master->get_digital(DIGITAL_A)) {
			lift_front_control->setTarget((3.0/8.0)*LIFT_GEAR_RATIO);
		}
		else if(master->get_digital(DIGITAL_B)) {
			lift_front_control->setTarget(100.0/360.0*LIFT_GEAR_RATIO);
		} else if(master->get_digital(DIGITAL_L1)){
			lift_front->moveVelocity(25);
		} else if(master->get_digital(DIGITAL_L2)) {
			lift_front->moveVelocity(-25);
		} else if(lift_front_control->isSettled()){
			lift_front->moveVelocity(0);
		}

		if(master->get_digital(DIGITAL_X)) {
			lift_back_control->setTarget((-3.0/8.0)*LIFT_GEAR_RATIO);
		}
		else if(master->get_digital(DIGITAL_Y)) {
			lift_back_control->setTarget(-100.0/360.0*LIFT_GEAR_RATIO);
		} else if(master->get_digital(DIGITAL_R1)){
			lift_back->moveVelocity(-25);
		} else if(master->get_digital(DIGITAL_R2)) {
			lift_back->moveVelocity(25);
		} else if(lift_back_control->isSettled()){
			lift_back->moveVelocity(0);
		}

		if(master->get_digital(DIGITAL_LEFT) && delay <= 0) {
			chassis_hold = !chassis_hold;
			delay = 200;
		}

		if(chassis_hold) {
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		}
		else {
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
		}

		if(chassis_hold) {

		}
		else {

		}

		// Intake
		if(master->get_digital(DIGITAL_UP)) {
			intake_flag = -1;
		}
		else if(master->get_digital(DIGITAL_DOWN)) {
			intake_flag = 1;
		}
		else if(master->get_digital(DIGITAL_RIGHT)) {
			intake_flag = 0;
		}
		if(intake_flag == 1) {
			intake->moveVoltage(-12000);
		} else if(intake_flag == -1){
			intake->moveVoltage(12000);
		}else {
			intake->moveVoltage(0);
		}
		pros::delay(20);
		if(delay > 0) {
			delay = delay - 20;
		}
	}
}