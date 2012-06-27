/*
 * Ardrone_localizer.h
 *
 *  Created on: May 16, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef ARDRONE_LOCALIZER_H_
#define ARDRONE_LOCALIZER_H_

#include "ros/ros.h"
#include "ds_control/Navdata.h"
#include "ds_control/Tag.h"
#include "ds_control/Tags.h"
#include "ds_control/marker.h"
#include "ds_control/EKF.h"
#include <Eigen/Core>

#include "geometry_msgs/Point.h"


class Ardrone_localizer {

public:

    Ardrone_localizer();

    float getX();
    float getY();
    float getYaw();
    float getHeight();

    void tagCB(const ds_control::TagsConstPtr& tag_msg);
    void navCB(const ds_control::NavdataConstPtr& nav_msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_nav;
    ros::Subscriber sub_tags;
    ros::Publisher pub_state;

    Eigen::Vector3f zeta_global_pose, beta_global_pose;

    ExtendedKalmanFilter kalman_filter;

    // visualization for the EKF-state and covariance
    EKF_marker ekf_marker;

    drone_marker markers;

    tf::Transform transform;

    ros::Time last_stamp;
    bool got_first_nav;
    float mean_height;
    int msg_cnt;
    float last_yaw;

private:

    float x;
    float y;
    float z;
    float phi;


};




#endif /* ARDRONE_LOCALIZER_H_ */
