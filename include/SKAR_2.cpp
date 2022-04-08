#include "SKAR_2.hpp"

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	selector::init();

	if (selector::auton == 0)
	{
		ks.kP = 0.002;
		ks.kI = 0;
		ks.kD = 0; //-0.00001;
		ks.kBias = 0;
	}
	else
	{
		ks.kP = 0.00064;
		ks.kI = 0;
		ks.kD = 0; //-0.00001;
		ks.kBias = 0;
	}

	// Drive Motors
	front_rt1.reset(new okapi::Motor(1, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_rt2.reset(new okapi::Motor(2, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt1.reset(new okapi::Motor(3, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt2.reset(new okapi::Motor(11, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft1.reset(new okapi::Motor(10, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft2.reset(new okapi::Motor(9, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_lft1.reset(new okapi::Motor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_lft2.reset(new okapi::Motor(8, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));

	front_rt.reset(new okapi::MotorGroup({front_rt1, front_rt2}));
	front_lft.reset(new okapi::MotorGroup({front_lft1, front_lft2}));
	back_rt.reset(new okapi::MotorGroup({back_rt1, back_rt2}));
	back_lft.reset(new okapi::MotorGroup({back_lft1, back_lft2}));

	drive_lft.reset(new okapi::MotorGroup({front_lft1, front_lft2, back_lft1, back_lft2}));
	drive_rt.reset(new okapi::MotorGroup({front_rt1, front_rt2, back_rt1, back_rt2}));
	chassis = okapi::ChassisControllerBuilder()
				  .withMotors(drive_lft, drive_rt)
				  // Green gearset, 4 in wheel diam, 11.5 in wheel track
				  .withDimensions({okapi::AbstractMotor::gearset::green, (3.0 / 5.0)}, {{3.25_in, 12.4375_in}, okapi::imev5GreenTPR})
				  .withGains(ks, ks)
				  .build();

	lift_front_lft.reset(new okapi::Motor(19, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front_rt.reset(new okapi::Motor(12, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front.reset(new okapi::MotorGroup({*lift_front_lft, *lift_front_rt}));
	lift_front->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	lift_front_control = okapi::AsyncPosControllerBuilder().withMotor(*lift_front).build();
	lift_front_control->setTarget(FRONT_LIFT_INIT);

	front_claw_piston.reset(new pros::ADIDigitalOut('A'));
	back_tilter.reset(new pros::ADIDigitalOut('B'));
	back_claw_piston.reset(new pros::ADIDigitalOut('C'));

	intake.reset(new okapi::Motor(13, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));

	intake->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	//camera.reset(new GoalCamera(17));

	imu.reset(new pros::Imu(15));

	dist_sensor.reset(new pros::Distance(18));

	master.reset(new pros::Controller(pros::E_CONTROLLER_MASTER));
	partner.reset(new pros::Controller(pros::E_CONTROLLER_PARTNER));
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
// original autonomous code
void autonomous()
{
	lift_front_control->tarePosition();
	chassis->stop();
	chassis->setMaxVelocity(200);
	if(selector::auton == 0) {
		// int move_vel = 150;	
		// chassis->setMaxVelocity(move_vel);

		// back_claw_control->setTarget(BACK_CLAW_DOWN);
		// pros::delay(250);
		// back_claw_control->setTarget(0);

		// // Grab Our Side
		// lift_front_control->setTarget(FRONT_LIFT_DOWN);
		// chassis->moveDistance(1.75_ft);
		// close_claw(front_claw_piston, front_claw_control);
		// lift_front_control->setTarget(FRONT_LIFT_MOVE);
		// intake->moveVoltage(INTAKE_IN);
		// chassis->moveDistance(5_ft);
		// intake->moveVoltage(0);

		// // Grab Opposing side on Balance
		// chassis->turnAngle(-40_deg);
		// chassis->moveDistance(1.5_ft);
		// lift_front_control->setTarget(FRONT_LIFT_PLAT);
		// pros::delay(1000);
		// chassis->turnAngle(-200_deg);
		// turn_to_goal(camera, drive_lft, drive_rt, BLUE);
		// chassis->moveDistance(0.75_ft);
		// lift_back_control->setTarget(BACK_LIFT_DOWN);
		// pros::delay(1000);
		// chassis->setMaxVelocity(50);
		// chassis->moveDistanceAsync(-2_ft);
		// pros::delay(3000);
		// chassis->stop();
		// chassis->setMaxVelocity(move_vel);
		// lift_back_control->setTarget(BACK_LIFT_UP);
		// back_claw_control->setTarget(BACK_CLAW_DOWN);
		// chassis->moveDistance(1.2_ft);

		// // Place Our Side
		// chassis->turnAngle(135_deg);
		// intake->moveVoltage(INTAKE_IN);
		// chassis->moveDistance(2.75_ft);
		// chassis->turnAngle(90_deg);
		// chassis->moveDistanceAsync(1.25_ft);
		// pros::delay(2000);
		// open_claw(front_claw_piston, front_claw_control);
		// pros::delay(1000);
		// intake->moveVoltage(0);

		// // Grab Yellow
		// chassis->moveDistance(-0.5_ft);
		// chassis->turnAngle(-100_deg);
		// chassis->moveDistance(-1.75_ft);
		// chassis->turnAngle(-80_deg);
		// lift_front_control->setTarget(FRONT_LIFT_DOWN);
		// pros::delay(1000);
		// chassis->moveDistance(3.5_ft);
		// close_claw(front_claw_piston, front_claw_control);
		// lift_front_control->setTarget(FRONT_LIFT_PLAT);

		// // Move to balance
		// chassis->moveDistance(4_ft);
		// chassis->turnAngle(-60_deg);
		// chassis->moveDistanceAsync(4_ft);
		// // pros::delay(3000);
		// // chassis->moveDistance(-2_ft);
		// // chassis->turnAngle(210_deg);
		// // chassis->moveDistance(1_ft);
		// // lift_front_control->setTarget(FRONT_LIFT_DOWN);
		// // balance(chassis, imu);
	}
	else
	{
		// Grab yellow
		// lift_front_control->setTarget(FRONT_LIFT_DOWN);
		// chassis->moveDistance(4.8_ft);
		// close_claw(front_claw_piston, front_claw_control);
		// //lift_front_control->setTarget(FRONT_LIFT_MOVE);
		// chassis->setMaxVelocity(150);

		//Grab Yellow
		int DIST = 35;
		drive_rt->moveVoltage(12000);
		drive_lft->moveVoltage(12000);
		lift_front_control->setTarget(FRONT_LIFT_MOVE);
		pros::delay(50);
		front_claw_piston->set_value(false);
		pros::delay(50);
		lift_front_control->setTarget(FRONT_LIFT_DOWN);
		int move_time = 100;

		drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		int MAX_TIME = 1400;
		while((dist_sensor->get() > DIST || dist_sensor->get() == 0) && move_time < MAX_TIME) {
			// if(dist_sensor->get() < 350 && dist_sensor->get() != 0) {
			// 	drive_rt->moveVoltage(6000);
			// 	drive_lft->moveVoltage(6000);
			// 	MAX_TIME = MAX_TIME+3;
			// }
			pros::delay(5);
			move_time += 5;
		}
		front_claw_piston->set_value(true);
		drive_rt->moveVoltage(0);
		drive_lft->moveVoltage(0);
		pros::delay(150);
		lift_front_control->setTarget(FRONT_LIFT_MOVE);

		// We're on the left
		if (abs(selector::auton) == 1)
		{
			chassis->moveDistance(-4.5_ft);
			lift_front_control->setTarget(FRONT_LIFT_PLAT);
			back_claw_piston->set_value(true);
			back_tilter->set_value(true);
			pros::delay(250);
			back_claw_piston->set_value(false);
			back_tilter->set_value(false);
			pros::delay(1000);
			chassis->turnAngle(-90_deg);
			// if (selector::auton < 0)
			// {
			// 	turn_to_goal(camera, drive_lft, drive_rt, BLUE);
			// }
			chassis->moveDistance(0.8_ft);
			pros::delay(1500);
			chassis->moveDistance(-2_ft);
			chassis->moveDistance(2_ft);
			chassis->turnAngle(-110_deg);
			chassis->moveDistance(-1.75_ft);
		}
		// We're on the right
		else
		{
			chassis->moveDistance(-3_ft);

			// Grab Blue

			back_claw_piston->set_value(true);
			back_tilter->set_value(true);
			pros::delay(250);
			back_claw_piston->set_value(false);
			back_tilter->set_value(false);
			chassis->turnAngle(-110_deg);
			chassis->waitUntilSettled();
			// if (selector::auton < 0)
			// {
			// 	turn_to_goal(camera, drive_lft, drive_rt, BLUE);
			// }
			chassis->moveDistance(1_ft);
			chassis->moveDistance(-2_ft);
			chassis->waitUntilSettled();
			chassis->moveDistance(1_ft);

			// Pick Up Rings
			chassis->turnAngle(-110_deg);
		}
		intake->moveVoltage(12000);
		lift_front_control->setTarget(FRONT_LIFT_PLAT);

		chassis->moveDistance(.75_ft);
		while (true)
		{
			chassis->moveDistance(1_ft);
			chassis->moveDistance(-1_ft);
		}
	}
}

// void autonomous()
// {
// 	//balance(chassis, imu);
// 	imu_turning(361, drive_rt, drive_lft, imu);
// }

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
	chassis->setMaxVelocity(200);
	int intake_flag = 0;
	int chassis_mode_delay = 0;
	int front_claw_timer = 0;
	int back_claw_timer = 0;
	bool front_flag = true;
	bool back_flag = true;
	bool chassis_hold = false;

	int back_timer = 0;
	int front_timer = 0;

	int double_tap = 0;
	int move_volt = 11000;
	while (true)
	{

		// Driving Mechanics
		double y = master->get_analog(ANALOG_LEFT_Y);
		double x = 0; // master->get_analog(ANALOG_LEFT_X);
		double z = -master->get_analog(ANALOG_RIGHT_X);

		front_rt->moveVoltage((y + x + z) / 127 * move_volt);
		back_rt->moveVoltage((y - x + z) / 127 * move_volt);
		front_lft->moveVoltage((y - x - z) / 127 * move_volt);
		back_lft->moveVoltage((y + x - z) / 127 * move_volt);

		if (master->get_digital(DIGITAL_R2))
		{
			lift_front_control->setTarget(FRONT_LIFT_DOWN);
			// front_claw_control->setTarget(0);
		}
		else if (master->get_digital(DIGITAL_R1))
		{
			lift_front_control->setTarget(FRONT_LIFT_PLAT);
			// front_claw_control->setTarget(1.0/4.0);
		}
		else if (master->get_digital(DIGITAL_L2))
		{
			lift_front->moveVelocity(-1000);
		}
		else if (master->get_digital(DIGITAL_L1))
		{
			lift_front->moveVelocity(1000);
		}
		else if (lift_front_control->isSettled())
		{
			lift_front->moveVelocity(0);
		}

		if (master->get_digital(DIGITAL_LEFT) && chassis_mode_delay <= 0)
		{
			chassis_hold = !chassis_hold;
			chassis_mode_delay = 200;
		}

		if (chassis_hold)
		{
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			move_volt = 6000;
		}
		else
		{
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			move_volt = 11000;
		}

		// Intake
		if (master->get_digital(DIGITAL_UP))
		{
			intake_flag = -1;
		}
		else if (master->get_digital(DIGITAL_DOWN))
		{
			intake_flag = 1;
		}
		else if (master->get_digital(DIGITAL_RIGHT))
		{
			intake_flag = 0;
			double_tap++;
		}
		if (intake_flag == 1)
		{
			intake->moveVoltage(-12000);
		}
		else if (intake_flag == -1)
		{
			intake->moveVoltage(12000);
		}
		else
		{
			intake->moveVoltage(0);
		}

		if (front_claw_timer <= 0 && master->get_digital(DIGITAL_B))
		{
			front_claw_timer = 300;
			front_flag = !front_flag;
		}		
		if (back_claw_timer <= 0 && master->get_digital(DIGITAL_A))
		{
			back_claw_timer = 300;
			back_flag = !back_flag;
		}

		front_claw_piston->set_value(front_flag);
		back_claw_piston->set_value(back_flag);

		// if(master->get_digital(DIGITAL_Y)) {
		// 	turn_to_goal(camera, drive_lft, drive_rt, AUTON_COLOR);
		// }

		// if(master->get_digital(DIGITAL_X)) {
		// 	balance(chassis, imu);
		// }

		pros::delay(20);

		if (chassis_mode_delay > 0)
		{
			chassis_mode_delay = chassis_mode_delay - 20;
		}
		if (front_claw_timer > 0)
		{
			front_claw_timer = front_claw_timer - 20;
		}
		if (back_claw_timer > 0)
		{
			back_claw_timer = back_claw_timer - 20;
		}
	}
}