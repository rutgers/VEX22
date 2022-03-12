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

void turn_to_goal(std::shared_ptr<GoalCamera> camera, 
                  std::shared_ptr<okapi::MotorGroup> lft, 
                  std::shared_ptr<okapi::MotorGroup> rt,
                  goal_color c) {    
    double x,y;
    std::tie(x, y) = camera->get_by_sig(c);
    
    int err_thresh = 20;
    int fov_angle = 60;   
    int settled_time = 0;
    int dt = 10;
    int settled_thresh = 0;
    int total_time = 0;
    double kp = 9000;
    double ki = 50;

    
    while(total_time < 2000 && (abs(x) > err_thresh || settled_thresh <= 150)) {
        std::tie(x, y) = camera->get_by_sig(c);
        double v_prop = x/VISION_FOV_WIDTH*2;
        double sign = 1;
        if(v_prop < 0){
            sign = -1;
        }
        lft->moveVoltage(-kp*v_prop - ki*dt*sign);
        rt->moveVoltage(kp*v_prop + ki*dt*sign);

        if(abs(x) < err_thresh) {
            settled_time += dt;
        }
        else {
            settled_time = 0;
        }
        pros::delay(dt);
        total_time += dt;
    }
    lft->moveVoltage(0);
    rt->moveVoltage(0);
}

void balance(std::shared_ptr<okapi::ChassisController> chassis, std::shared_ptr<pros::Imu> imu) {
    double orig_velocity = chassis->getMaxVelocity();
    chassis->setMaxVelocity(100);
    double original_pitch = imu->get_roll();
    chassis->moveDistanceAsync(12_ft);
    
    double pitch_change_thresh = 10;

    while (abs(imu->get_roll() - original_pitch) < pitch_change_thresh) {
        pros::delay(20);
    }
    
    chassis->moveDistanceAsync(26.5_in);
    original_pitch = imu->get_roll();

    while (abs(imu->get_roll() - original_pitch) < pitch_change_thresh) {
        pros::delay(20);
    }
    chassis->stop();
    chassis->setMaxVelocity(orig_velocity);
}   