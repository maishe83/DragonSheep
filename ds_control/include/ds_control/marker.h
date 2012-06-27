/*
 * marker.h
 *
 *  Created on: Apr 18, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef MARKER_H_
#define MARKER_H_


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <Eigen/Core>

class drone_marker {

public:

    void publish_markers();

    drone_marker();
    void init();
    void addMarkerPose(const tf::StampedTransform& trafo);

    std::vector<tf::StampedTransform> trafos;

    ros::NodeHandle nh_;
    ros::Publisher pub_markers;
    tf::TransformBroadcaster br;

};


class EKF_marker {

public:

    void publish_last_n_states(int n = -1);

    EKF_marker();

    void init();
    void addFilterState(Eigen::Vector3f mu, Eigen::Matrix3f sigma, float height = 0);

    ros::NodeHandle nh_;
    ros::Publisher pub_markers;

private:
    std::vector<Eigen::Vector3f> mus;
    std::vector<Eigen::Matrix3f> sigmas;
    std::vector<float> heights;

};



#endif /* MARKER_H_ */
