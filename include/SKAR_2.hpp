#ifndef MAIN_H
#define MAIN_H
#include "main.h"
#endif
#ifndef OKAPI_H
#define OKAPI_H
#include "okapi/api.hpp"
#endif
#ifndef VISION_CPP
#define VISION_CPP
#include "vision.cpp"
#endif
#ifndef AUTON_CPP
#define AUTON_CPP
#include "auton_util.cpp"
#endif


float BACK_LIFT_GEAR_RATIO = 5.0/1.0;
float FRONT_LIFT_GEAR_RATIO = 7.0/1.0;
float CHASSIS_GEAR_RATIO = 3.0/5.0;
double FRONT_LIFT_PLAT_PLACE = 70.0/360.0*FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_PLAT = 90.0/360.0*FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_DOWN = 1.0/360.0*FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_MOVE = 30.0/360.0*FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_INIT = -5.0/360.0*FRONT_LIFT_GEAR_RATIO;

double BACK_LIFT_DOWN = 90/360.0*BACK_LIFT_GEAR_RATIO;
double BACK_LIFT_UP = 51.0/360.0*BACK_LIFT_GEAR_RATIO;

double BACK_CLAW_DOWN = -100.0/360.0;
double FRONT_CLAW_DOWN = (135.0/360.0);
int INTAKE_IN = 8000;
int INTAKE_OUT = -8000;

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
std::shared_ptr<okapi::AsyncPositionController<double, double>> front_claw_control;

std::shared_ptr<GoalCamera> camera;
 
std::shared_ptr<pros::Imu> imu;

std::shared_ptr<pros::Distance> dist_sensor;

std::shared_ptr<pros::Controller> master;
std::shared_ptr<pros::Controller> partner;