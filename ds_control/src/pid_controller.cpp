/*
 * pid_controller.cpp
 *
 *  Created on: May 16, 2012
 *      Author: Nikolas Engelhard
 */


#include "ds_control/pid_controller.h"

PID_Controller::PID_Controller(){
    c_proportional = c_integral = c_derivative = error_sum = 0;
    last_error = std::numeric_limits<float>::max();
}

float PID_Controller::getCommand(float error, ros::Time current){

    float dt = (last_time-current).toSec();


    float P = c_proportional * (error);
    float I = c_integral * error_sum;
    float D = c_derivative * ((last_error-error)    / dt);

    last_time  = current;
    last_error = error;
    error_sum += error * dt;

    return P-I+D;
}
