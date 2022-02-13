#include "main.h"
#include "okapi/api.hpp"
#include <vector>

okapi::IterativePosPIDController::Gains ks; 

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

std::shared_ptr<okapi::ChassisController> chassis;

std::shared_ptr<okapi::Motor> lift_front_lft;
std::shared_ptr<okapi::Motor> lift_front_rt;
std::shared_ptr<okapi::MotorGroup> lift_front;
std::shared_ptr<okapi::AsyncPositionController<double, double>> lift_front_control;

std::shared_ptr<okapi::Motor> lift_back_lft;
std::shared_ptr<okapi::Motor> lift_back_rt;
std::shared_ptr<okapi::MotorGroup> lift_back;
std::shared_ptr<okapi::AsyncPositionController<double, double>> lift_back_control;

std::shared_ptr<pros::Controller> master;
std::shared_ptr<pros::ADIDigitalOut> piston;