/*
 * pid_controller.h
 *
 *  Created on: May 16, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_
#include <ros/ros.h>
#include <limits>

struct PID_Controller {


    float c_proportional;
    float c_integral;
    float c_derivative;

    // calculation of new command
    float getCommand(float error, ros::Time current);


    PID_Controller();

private:

    // useful for d-part
    ros::Time last_time;
    float last_error;

    // useful for i-part
    float error_sum;


};



#endif /* PDI_CONTROLLER_H_ */
