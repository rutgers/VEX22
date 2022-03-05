#include "main.h"
#include "okapi/api.hpp"
#include <vector>

okapi::IterativePosPIDController::Gains ks; 

std::shared_ptr<okapi::Motor> frontFrontLft;
std::shared_ptr<okapi::Motor> frontLft;
std::shared_ptr<okapi::Motor> backLft;
std::shared_ptr<okapi::Motor> backBackLft;

std::shared_ptr<okapi::Motor> frontFrontRt;
std::shared_ptr<okapi::Motor> frontRt;
std::shared_ptr<okapi::Motor> backRt;
std::shared_ptr<okapi::Motor> backBackRt;

std::shared_ptr<okapi::MotorGroup> drive_lft;
std::shared_ptr<okapi::MotorGroup> drive_rt;

std::shared_ptr<okapi::ChassisController> chassis;

std::shared_ptr<okapi::Motor> backLeftLift;
std::shared_ptr<okapi::Motor> backRtLift;
std::shared_ptr<okapi::MotorGroup> back_lift;

std::shared_ptr<okapi::Motor> frontLftLift;
std::shared_ptr<okapi::Motor> frontRtLift;
std::shared_ptr<okapi::MotorGroup> front_lift;

std::shared_ptr<okapi::Motor> lift_back_lft;
std::shared_ptr<okapi::Motor> lift_back_rt;
std::shared_ptr<okapi::MotorGroup> lift_back;
std::shared_ptr<okapi::AsyncPositionController<double, double>> lift_back_control;

std::shared_ptr<pros::Controller> master;
std::shared_ptr<pros::ADIDigitalOut> piston;