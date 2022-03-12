#include "SKAR_1.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

int FRONT_LIFT_GEAR_RATIO = 5;
int BACK_LIFT_GEAR_RATIO = 7;

double BACK_LIFT_DOWN = (1.5 / 5.0) * BACK_LIFT_GEAR_RATIO;
double BACK_LIFT_UP = (0.03) * BACK_LIFT_GEAR_RATIO;
double BACK_LIFT_UP_S = (1.5 / 6.0) * BACK_LIFT_GEAR_RATIO;

double FRONT_LIFT_DOWN = (0.05) * FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_UP = (0.27) * FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_UP_S = (0.35) * FRONT_LIFT_GEAR_RATIO;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few 1.seconds.
 */
void initialize()
{

	// PID Variables
	ks.kP = 0.00101;
	ks.kI = 0;
	ks.kD = 0;
	ks.kBias = 0;

	// Drive Motors
	frontFrontLft.reset(new okapi::Motor(15, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	frontLft.reset(new okapi::Motor(3, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	backLft.reset(new okapi::Motor(5, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
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
				  .withDimensions(okapi::AbstractMotor::gearset::green, {{3.25_in, 14.5_in}, okapi::imev5GreenTPR})
				  .withGains(ks, ks)
				  .build();

	// Back Lift
	backLeftLift.reset(new okapi::Motor(11, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	backRtLift.reset(new okapi::Motor(20, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	back_lift.reset(new okapi::MotorGroup({backLeftLift, backRtLift}));
	back_lift->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	back_lift_control = okapi::AsyncPosControllerBuilder().withMotor(back_lift).build();

	// Front Lift
	frontLftLift.reset(new okapi::Motor(18, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	frontRtLift.reset(new okapi::Motor(13, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	front_lift.reset(new okapi::MotorGroup({frontLftLift, frontRtLift}));
	front_lift->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	front_lift_control = okapi::AsyncPosControllerBuilder().withMotor(front_lift).build();

	// intake
	intake.reset(new okapi::Motor(4, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));

	// Controller Initialization
	master.reset(new pros::Controller(pros::E_CONTROLLER_MASTER));

	// Pneumatics
	piston.reset(new pros::ADIDigitalOut('A'));

	// Camera
	camera.reset(new GoalCamera(7));

	// IMU
	imu.reset(new pros::Imu(17));
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

void auton_small_side()
{
	// Move to the Yellow
	chassis->setMaxVelocity(AUTON_INIT);
	chassis->moveDistance(2.75_ft);
	chassis->setMaxVelocity(165);

	// Grab the Yellow
	// pros::delay(0);
	piston->set_value(true);
	// pros::delay(400);

	// Move back to the original position
	chassis->moveDistance(-2.75_ft);
	pros::delay(400);

	// Lift the Front Lift with the Yellow
	front_lift_control->setTarget(FRONT_LIFT_UP);
	pros::delay(400);

	// Turn to the blue
	chassis->turnAngle(-70_deg);
	pros::delay(500);

	// Move back for the back lift space
	front_lift_control->setTarget(FRONT_LIFT_UP);
	pros::delay(100);
	chassis->setMaxVelocity(90);
	chassis->moveDistanceAsync(0.9_ft);
	pros::delay(800);
	chassis->stop();

	// Bring the Back Lift Down
	back_lift_control->setTarget(BACK_LIFT_DOWN);
	pros::delay(1000);

	// Move the chassis towards the blue
	if (AUTON_COLOR == BLUE)
	{
		turn_to_goal(camera, drive_rt, drive_lft, AUTON_COLOR);
	}
	chassis->moveDistanceAsync(-1_ft);
	pros::delay(1000);
	chassis->stop();

	// Lift the blue up
	back_lift_control->setTarget(BACK_LIFT_UP);
	chassis->setMaxVelocity(90);
	pros::delay(300);

	// Move back to get rings
	chassis->moveDistance(0.2_ft);

	// Turn around to put rings inside
	chassis->turnAngle(60_deg);
	pros::delay(200);

	// Move toward yellow again
	chassis->moveDistance(0.2_ft);
	// piston->set_value(true);

	// Turn around to put rings
	pros::delay(100);
	chassis->turnAngle(140_deg);

	// Start moving intake
	intake->moveVelocity(100);

	// Storing rings time
	while (true)
	{
		chassis->moveDistance(-0.6_ft);
		chassis->moveDistance(0.6_ft);
	}
}

void autonomous()
{
	// Back Lift Gear Ratio: 1:7
	// maxVelocity 165
	if (SKILLS == true) // SKILLS CODE
	{

		// Drive Forward
		chassis->moveDistance(0.2_ft);
		pros::delay(400);

		// Grab Red
		piston->set_value(true);
		pros::delay(400);

		// Lift Up the Red
		front_lift_control->setTarget(FRONT_LIFT_UP);
		pros::delay(300);

		// Move Back to Turn Around
		chassis->moveDistance(-0.4_ft);
		pros::delay(200);

		// Move Back Lift Down
		back_lift_control->setTarget(BACK_LIFT_DOWN);
		pros::delay(200);

		// Turn Towards the Yellow
		chassis->turnAngle(60_deg);
		pros::delay(200);
		turn_to_goal(camera, drive_rt, drive_lft, SKILLS_COLOR);
		pros::delay(200);

		// Move toward the the Yellow
		chassis->moveDistance(-2.75_ft);
		pros::delay(200);

		// Move Back Lift Up
		back_lift_control->setTarget(BACK_LIFT_UP_S);
		pros::delay(200);

		// Move slight
		chassis->moveDistance(-1_ft);
		pros::delay(200);

		// Turn toward the right
		chassis->turnAngle(-105_deg);
		chassis->waitUntilSettled();

		// Move chassis back
		back_lift_control->setTarget(BACK_LIFT_DOWN);
		pros::delay(20000);
		chassis->moveDistance(1_ft);
		pros::delay(200);

		// Move the chassis forward slighly
		chassis->moveDistance(0.2_ft);
		pros::delay(200);

		// Drop the Red
		piston->set_value(false);
		pros::delay(200);
	}
	else // AUTON CODE
	{
		if (AUTON_NUM == 1)
		{
			auton_small_side();
		}
		else
		{
		}
	}
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
void opcontrol()
{
	chassis->stop();
	int piston_timer = 0;
	bool piston_flag = false;

	while (true)
	{

		/*
			Driving Controls:

			Left Joystick Up and Down - Control Left Side of Chassis
			Right Joystick Up and Down - Control Right Side of Chassis
			R1 - Back Lift Up
			R2 - Back Lift Down
			L1 - Front Lift Up
			L2 - Front Lift Down
			Right - Piston Up and Down
			Up (Toggle) - Intake Up
			Down (Toggle) - Intake Down
			Right - Toggle Up and Down Off
		*/

		// Drive Mechanics

		if (TANK == false)
		{
			double y = master->get_analog(ANALOG_LEFT_Y);
			double x = 0; // master->get_analog(ANALOG_LEFT_X);
			double z = -master->get_analog(ANALOG_RIGHT_X);
			drive_lft->moveVelocity((y + x - z) * 100);
			drive_rt->moveVelocity((y - x + z) * 100);
		}
		else
		{
			double y_l = master->get_analog(ANALOG_LEFT_Y);
			double y_r = master->get_analog(ANALOG_RIGHT_Y);
			drive_lft->moveVelocity(y_l);
			drive_rt->moveVelocity(y_r);
		}

		// Back Lifting Mechanics
		if (master->get_digital(DIGITAL_R1))
		{ // Up
			back_lift->moveVelocity(-150);
		}
		else if (master->get_digital(DIGITAL_R2))
		{ // Down
			back_lift->moveVelocity(150);
		}
		else
		{
			back_lift->moveVelocity(0);
		}

		// Front Lifting Mechanics
		if (master->get_digital(DIGITAL_L1))
		{ // Up
			front_lift->moveVelocity(1000);
		}
		else if (master->get_digital(DIGITAL_L2))
		{ // Down
			front_lift->moveVelocity(-1000);
		}
		else
		{
			front_lift->moveVelocity(0);
		}

		// Front Lift Piston Mechanics
		if (piston_timer <= 0 && master->get_digital(DIGITAL_A))
		{
			piston_timer = 200;
			piston_flag = !piston_flag;
		}
		piston->set_value(piston_flag);
		if (piston_timer > 0)
		{
			piston_timer = piston_timer - 20;
		}

		// Intake Mechanics
		if (master->get_digital(DIGITAL_UP))
		{ // Intake Up
			intake->moveVelocity(1200);
		}
		else if (master->get_digital(DIGITAL_DOWN))
		{ // Intake Down
			intake->moveVelocity(-1200);
		}
		else if (master->get_digital(DIGITAL_LEFT))
		{
			intake->moveVelocity(0);
		}

		// Brake Mode
		if (master->get_digital(DIGITAL_X))
		{
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		}

		pros::delay(20);
	}
}