#include "main.h"
#include "okapi/api.hpp"
#include <vector>

// Tank Control Swap
#define TANK 1

// PID Control
okapi::IterativePosPIDController::Gains ks; 

// Drive Motor Initializations
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

// Chassis Initialization
std::shared_ptr<okapi::ChassisController> chassis;

// Back Lift Initializations
std::shared_ptr<okapi::Motor> backLeftLift;
std::shared_ptr<okapi::Motor> backRtLift;
std::shared_ptr<okapi::MotorGroup> back_lift;
std::shared_ptr<okapi::AsyncPositionController<double, double>> back_lift_control;

// Front Lift Initializations
std::shared_ptr<okapi::Motor> frontLftLift;
std::shared_ptr<okapi::Motor> frontRtLift;
std::shared_ptr<okapi::MotorGroup> front_lift;

// Intake Initializations
std::shared_ptr<okapi::Motor> intake;

// Controller Initializations
std::shared_ptr<pros::Controller> master;

// Piston Initializations
std::shared_ptr<pros::ADIDigitalOut> piston;