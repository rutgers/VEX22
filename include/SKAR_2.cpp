#include "SKAR_2.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void close_claw(std::shared_ptr<pros::ADIDigitalOut> piston, std::shared_ptr<okapi::AsyncPositionController<double, double>> control) {
	piston->set_value(false);
	control->setTarget(0.25);
	pros::delay(100);
	piston->set_value(true);
}

void open_claw(std::shared_ptr<pros::ADIDigitalOut> piston, std::shared_ptr<okapi::AsyncPositionController<double, double>> control) {
	piston->set_value(false);
	control->setTarget(0);
	pros::delay(100);
	piston->set_value(true);
}

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
	ks.kP = 0.0010;
	ks.kI = 0;
	ks.kD = -0.000002;
	ks.kBias = 0;

	// Drive Motors
	front_rt1.reset(new okapi::Motor(12, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_rt2.reset(new okapi::Motor(11, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt1.reset(new okapi::Motor(2, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt2.reset(new okapi::Motor(1, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft1.reset(new okapi::Motor(19, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft2.reset(new okapi::Motor(20, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
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
				  .withDimensions({okapi::AbstractMotor::gearset::green, (5.0/3.0)}, {{3.25_in, 12.5_in}, okapi::imev5GreenTPR})
				  .withGains(ks, ks)
				  .build();

	lift_front_lft.reset(new okapi::Motor(18, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front_rt.reset(new okapi::Motor(13, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front.reset(new okapi::MotorGroup({*lift_front_lft, *lift_front_rt}));
	lift_front->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	lift_front_control = okapi::AsyncPosControllerBuilder().withMotor(*lift_front).build();

	lift_back_lft.reset(new okapi::Motor(8, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_back_rt.reset(new okapi::Motor(3, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_back.reset(new okapi::MotorGroup({*lift_back_lft, *lift_back_rt}));
	lift_back->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	lift_back_control = okapi::AsyncPosControllerBuilder().withMotor(lift_back).build();

	back_claw_lft.reset(new okapi::Motor(7, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_claw_rt.reset(new okapi::Motor(4, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_claw.reset(new okapi::MotorGroup({*back_claw_lft, *back_claw_rt}));
	back_claw->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	back_claw_control = okapi::AsyncPosControllerBuilder().withMotor(back_claw).build();

	front_claw_piston.reset(new pros::ADIDigitalOut('A'));
	front_claw_motor.reset(new okapi::Motor(14, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	front_claw_control = okapi::AsyncPosControllerBuilder().withMotor(front_claw_motor).build();

	intake_lft.reset(new okapi::Motor(6, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	intake_rt.reset(new okapi::Motor(5, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	intake.reset(new okapi::MotorGroup({*intake_lft, *intake_rt}));

	camera.reset(new GoalCamera(17));

	imu.reset(new pros::Imu(15));

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
void autonomous()
{
	chassis->stop();
	if(SKILLS) {
		double orig_vel = chassis->getMaxVelocity();
		chassis->setMaxVelocity(100);
		chassis->moveDistance(1_ft);
		close_claw(front_claw_piston, front_claw_control);
		lift_front_control->setTarget(FRONT_LIFT_MOVE);	
		intake->moveVoltage(INTAKE_IN);
		chassis->moveDistance(7_ft);
		intake->moveVoltage(0);
		chassis->turnAngle(-45_deg);
		chassis->moveDistance(3_ft);
		lift_back_control->setTarget(BACK_LIFT_DOWN);
		chassis->turnAngle(-180_deg);
		turn_to_goal(RED);
		chassis->moveDistance(2_ft);
		lift_back_control->setTarget(BACK_LIFT_UP);
		chassis->moveDistance(-2_ft);
		chassis->turnAngle(180_deg);
		lift_front_control->setTarget(FRONT_LIFT_MAX);
		chassis->moveDistance(2_ft);
		lift_front_control->setTarget(FRONT_LIFT_PLAT);
		open_claw(front_claw_piston, front_claw_control);
		chassis->turnAngle(-135_deg);
		lift_front_control->setTarget(FRONT_LIFT_DOWN);
		chassis->moveDistance(4_ft);
		close_claw(front_claw_piston, front_claw_control);
		lift_front_control->setTarget(FRONT_LIFT_PLAT);
		chassis->moveDistance(4_ft);
		chassis->turnAngle(-45_deg);
		chassis->moveDistance(2_ft);
		chassis->turnAngle(135_deg);
		chassis->moveDistance(2_ft);
		lift_front_control->setTarget(FRONT_LIFT_DOWN);
		balance(chassis, imu);
		chassis->setMaxVelocity(orig_vel);
	}
	else {
		// //Lift Gear ratio 1:5
		lift_front_control->setTarget((3.0 / 8.0) * FRONT_LIFT_GEAR_RATIO);
		chassis->moveDistance(6_ft);
		lift_front_control->setTarget(100.0 / 360.0 * FRONT_LIFT_GEAR_RATIO);
		pros::delay(1000);
		lift_front_control->waitUntilSettled();
		chassis->moveDistance(-4_ft);
		chassis->waitUntilSettled();
		chassis->moveDistance(1_ft);
		chassis->waitUntilSettled();
		lift_back_control->setTarget(-3.0 / 8.0 * BACK_LIFT_GEAR_RATIO);
		chassis->turnAngle(-140_deg);
		chassis->waitUntilSettled();
		chassis->moveDistance(-3_ft);
		chassis->waitUntilSettled();
		lift_back_control->setTarget(-100.0 / 360.0 * BACK_LIFT_GEAR_RATIO);
		chassis->moveDistance(3_ft);
		intake->moveVoltage(12000);
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
	int intake_flag = 0;
	int delay = 0;
	int front_claw_timer = 0;
	int back_claw_timer = 0;
	bool front_flag = false;
	bool back_flag = true;
	bool chassis_hold = false;

	int piston_timer = 0;
	bool piston_flag = false;

	int double_tap = 0;

	while (true)
	{

		// Driving Mechanics
		double y = master->get_analog(ANALOG_LEFT_Y);
		double x = 0;//master->get_analog(ANALOG_LEFT_X);
		double z = -master->get_analog(ANALOG_RIGHT_X);

		front_rt->moveVoltage((y + x + z) / 127 * 11000);
		back_rt->moveVoltage((y - x + z) / 127 * 11000);
		front_lft->moveVoltage((y - x - z) / 127 * 11000);
		back_lft->moveVoltage((y + x - z) / 127 * 11000);

		if(master->get_digital(DIGITAL_L2)) {
			lift_back_control->setTarget((2.0/8.0)*BACK_LIFT_GEAR_RATIO);
		}
		else if(master->get_digital(DIGITAL_L1)) {
			lift_back_control->setTarget(45.0/360.0*BACK_LIFT_GEAR_RATIO);
		} else 
		if(partner->get_digital(DIGITAL_L2)){
			lift_back->moveVelocity(1000);
		}
		else if (partner->get_digital(DIGITAL_L1))
		{
			lift_back->moveVelocity(-1000);
		}
		else if (lift_back_control->isSettled())
		{
			lift_back->moveVelocity(0);
		}

		if(master->get_digital(DIGITAL_R2)) {
			lift_front_control->setTarget((-3.0/8.0)*BACK_LIFT_GEAR_RATIO);
			// front_claw_control->setTarget(0);
		}
		else if(master->get_digital(DIGITAL_R1)) {
			lift_front_control->setTarget(-100.0/360.0*BACK_LIFT_GEAR_RATIO);
			// front_claw_control->setTarget(1.0/4.0);
		}
		else 
		if (partner->get_digital(DIGITAL_R2))
		{
			lift_front->moveVelocity(-1000);
		}
		else if (partner->get_digital(DIGITAL_R1))
		{
			lift_front->moveVelocity(1000);
		}
		else if (lift_front_control->isSettled())
		{
			lift_front->moveVelocity(0);
		}

		if (master->get_digital(DIGITAL_LEFT) && delay <= 0)
		{
			chassis_hold = !chassis_hold;
			delay = 200;
		}

		if (chassis_hold)
		{
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		}
		else
		{
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
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

		if(front_claw_timer <= 0 && master->get_digital(DIGITAL_B)) {
			front_claw_piston->set_value(false);

			if(!front_flag) {
				front_claw_control->setTarget(0.25);
			}
			else {
				front_claw_control->setTarget(0);
			}
			front_claw_timer = 300;
			front_flag = !front_flag;
		}

		if(back_claw_timer <= 0 && master->get_digital(DIGITAL_A)) {
			if(!back_flag) {
				back_claw_control->setTarget(0.25);
			}
			else {
				back_claw_control->setTarget(0);
			}
			back_claw_timer = 300;
			back_flag = !back_flag;
		}
		
		if(front_claw_control->isSettled()){
			front_claw_piston->set_value(true);
		}

		if(master->get_digital(DIGITAL_Y)) {
			turn_to_goal(camera, drive_lft, drive_rt, BLUE);
		}

		if(master->get_digital(DIGITAL_X)) {
			balance(chassis, imu, CHASSIS_GEAR_RATIO);
		}

		pros::delay(20);

		if (delay > 0)
		{
			delay = delay - 20;
		}
		if(front_claw_timer > 0) {
			front_claw_timer = front_claw_timer - 20;
		} 
		if(back_claw_timer > 0) {
			back_claw_timer = back_claw_timer - 20;
		}   

	}
}