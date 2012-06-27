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


struct Ardrone_localizer {

    ros::NodeHandle nh_;
    ros::Subscriber sub_nav;
    ros::Subscriber sub_tags;
    ros::Publisher pub_state;

    Eigen::Vector3f zeta_global_pose, beta_global_pose;

    ExtendedKalmanFilter kalman_filter;

    float getX(){return kalman_filter.state(0);}
    float getY(){return kalman_filter.state(1);}
    float getYaw(){return kalman_filter.state(2);}
    float getHeight(){return z;}

    EKF_marker ekf_marker;   // visualization for the EKF-state and covariance

    drone_marker markers;
    tf::Transform transform;

    ros::Time last_stamp;
    bool got_first_nav;

    float mean_height;
    int msg_cnt;

    double last_yaw;


    void tagCB(const ds_control::TagsConstPtr& tag_msg);
    void navCB(const ds_control::NavdataConstPtr& nav_msg);

    Ardrone_localizer();

private:
    float x,y,z,phi;


};




#endif /* ARDRONE_LOCALIZER_H_ */
