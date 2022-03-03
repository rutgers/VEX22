#include "main.h"
#include "okapi/api.hpp"
#include <vector>
int LIFT_GEAR_RATIO = 5;

std::shared_ptr<okapi::Motor> front_rt1;
std::shared_ptr<okapi::Motor> front_rt2;
std::shared_ptr<okapi::Motor> back_rt1;
std::shared_ptr<okapi::Motor> back_rt2;
std::shared_ptr<okapi::Motor> front_lft1;
std::shared_ptr<okapi::Motor> front_lft2;
std::shared_ptr<okapi::Motor> back_lft1;
std::shared_ptr<okapi::Motor> back_lft2;

std::shared_ptr<okapi::MotorGroup> front_rt;
std::shared_ptr<okapi::MotorGroup> back_rt;
std::shared_ptr<okapi::MotorGroup> front_lft;
std::shared_ptr<okapi::MotorGroup> back_lft;

std::shared_ptr<okapi::MotorGroup> drive_lft;
std::shared_ptr<okapi::MotorGroup> drive_rt;

okapi::IterativePosPIDController::Gains ks;
std::shared_ptr<okapi::ChassisController> chassis;

std::shared_ptr<okapi::Motor> lift_front_lft;
std::shared_ptr<okapi::Motor> lift_front_rt;
std::shared_ptr<okapi::MotorGroup> lift_front;
std::shared_ptr<okapi::AsyncPositionController<double, double>> lift_front_control;

std::shared_ptr<okapi::Motor> lift_back_lft;
std::shared_ptr<okapi::Motor> lift_back_rt;
std::shared_ptr<okapi::MotorGroup> lift_back;
std::shared_ptr<okapi::AsyncPositionController<double, double>> lift_back_control;

std::shared_ptr<okapi::Motor> back_claw_lft;
std::shared_ptr<okapi::Motor> back_claw_rt;
std::shared_ptr<okapi::MotorGroup> back_claw;
std::shared_ptr<okapi::AsyncPositionController<double, double>> back_claw_control;

std::shared_ptr<okapi::Motor> intake_lft;
std::shared_ptr<okapi::Motor> intake_rt;
std::shared_ptr<okapi::MotorGroup> intake;

std::shared_ptr<pros::ADIDigitalOut> front_claw_piston;
std::shared_ptr<okapi::Motor> front_claw_motor;

std::shared_ptr<pros::Vision> camera;
 
std::shared_ptr<pros::Imu> imu;

std::shared_ptr<pros::Controller> master;