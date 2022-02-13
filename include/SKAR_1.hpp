#include "main.h"
#include "okapi/api.hpp"
#include <vector>
int LIFT_GEAR_RATIO = 5;

std::shared_ptr<okapi::Motor> front_rt;
std::shared_ptr<okapi::Motor> back_rt;
std::shared_ptr<okapi::Motor> front_lft;
std::shared_ptr<okapi::Motor> back_lft;
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

std::shared_ptr<okapi::Motor> intake;

std::shared_ptr<pros::Controller> master;