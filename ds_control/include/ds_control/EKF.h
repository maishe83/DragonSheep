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

class ExtendedKalmanFilter {

public:

    // x, y, yaw
    Eigen::Vector3f state;
    // uncertainty of state
    Eigen::Matrix3f sigma;

    // process noise
    Eigen::Matrix3f Q;
    // observation noise
    Eigen::Matrix3f R;

    // x_{t+1} = g(x_t,u) and update uncertainty
    void predictionStep(const Eigen::Vector3f& odometry);

    // compare expected and measured values, update state and uncertainty
    void correctionStep(const Eigen::Vector3f& measurement,
                        const Eigen::Vector3f& global_marker_pose);

    void printState();
    void initFilter();

    EKF_marker marker;

    void testFilter();

    Eigen::MatrixXf createTestOdometry();
    Eigen::MatrixXf createTestMeasurements();
};


#endif /* EKF_H_ */
