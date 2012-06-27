/*
 * EKF.h
 *
 *  Created on: May 7, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include "ds_control/marker.h"

struct ExtendedKalmanFilter {

    Eigen::Vector3f state; // x, y, yaw
    Eigen::Matrix3f sigma; // uncertainty of state

    Eigen::Matrix3f Q; // process noise
    Eigen::Matrix3f R; // observation noise

    void predictionStep(const Eigen::Vector3f& odometry); // x_{t+1} = g(x_t,u) and update uncertainty
    void correctionStep(const Eigen::Vector3f& measurement, const Eigen::Vector3f& global_marker_pose);  // compare expected and measured values, update state and uncertainty

    void printState(){
        std::cout << "kalman state: " << state(0) << "  " << state(1) << " " << state(2)/M_PI*180 << std::endl;
    }

    void initFilter(){
        state =  Eigen::Vector3f(0,0,0);
        sigma = Eigen::Matrix3f::Zero(); sigma(0,0) = sigma(1,1) = 1; sigma(2,2) = 1;
        Q = Eigen::Matrix3f::Zero();     Q(0,0) = Q(1,1) = 0.002; Q(2,2) = 0.00002;
        R = Eigen::Matrix3f::Zero();     R(0,0) = R(1,1) = 0.01; R(2,2) = 0.0001;
    }


    EKF_marker marker;

    void testFilter();

    Eigen::MatrixXf createTestOdometry();
    Eigen::MatrixXf createTestMeasurements();


};


#endif /* EKF_H_ */
