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

int FRONT_LIFT_GEAR_RATIO = 7;
int BACK_LIFT_GEAR_RATIO = 7;

double BACK_LIFT_DOWN = (1.5 / 5.0) * BACK_LIFT_GEAR_RATIO;
double BACK_LIFT_UP = -(0.5 / 8.0) * BACK_LIFT_GEAR_RATIO;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	// PID Variables
	ks.kP = 0.0016;
	ks.kI = 0;
	ks.kD = 0;
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

	// intake
	intake.reset(new okapi::Motor(4, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));

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

void autonomous()
{
	// Back Lift Gear Ratio: 1:7
	// maxVelocity 165
	chassis->setMaxVelocity(165);
	chassis->moveDistance(3_ft);
	pros::delay(300);
	piston->set_value(true);
	chassis->moveDistance(-3_ft);
	pros::delay(300);
	piston->set_value(false);
	pros::delay(200);
	chassis->turnAngle(-70_deg);
	pros::delay(200);
	chassis->moveDistance(1_ft);
	back_lift_control->setTarget(BACK_LIFT_DOWN);
	pros::delay(200);
	/* back_lift_control->setTarget(BACK_LIFT_UP);
	pros::delay(300);
	chassis->moveDistance(5.5_ft);
	chassis->turnAngle(80_deg);
	chassis->moveDistance(1_ft);
	piston->set_value(true); */
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

		if (TANK == 1)
		{
			double y = master->get_analog(ANALOG_LEFT_Y);
			double x = 0; // master->get_analog(ANALOG_LEFT_X);
			double z = -master->get_analog(ANALOG_RIGHT_X);
			drive_lft->moveVelocity(y + x - z);
			drive_rt->moveVelocity(y - x + z);
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
			back_lift->moveVelocity(-50);
		}
		else if (master->get_digital(DIGITAL_R2))
		{ // Down
			back_lift->moveVelocity(50);
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
			intake->moveVelocity(700);
		}
		else if (master->get_digital(DIGITAL_DOWN))
		{ // Intake Down
			intake->moveVelocity(-700);
		}
		else if (master->get_digital(DIGITAL_LEFT))
		{
			intake->moveVelocity(0);
		}

		pros::delay(20);
	}
}