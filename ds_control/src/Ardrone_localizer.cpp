/*
 * Ardrone_localizer.cpp
 *
 *  Created on: May 16, 2012
 *      Author: engelhan
 */

#include "ds_control/Ardrone_localizer.h"



Ardrone_localizer::Ardrone_localizer(){
    got_first_nav = false;
    kalman_filter.initFilter();

    x = y = z = phi = 0;

    // pose of Marker in global coordinates
    zeta_global_pose = Eigen::Vector3f(0,0,0); // in meters
    beta_global_pose = Eigen::Vector3f(1,0,0);

    msg_cnt = mean_height = 0;

    sub_tags  = nh_.subscribe("/tags",100,&Ardrone_localizer::tagCB, this);
    sub_nav   = nh_.subscribe("/ardrone/navdata",100,&Ardrone_localizer::navCB, this);
    pub_state = nh_.advertise<geometry_msgs::Point>("/quadcopter_state",10);
}


void Ardrone_localizer::tagCB(const ds_control::TagsConstPtr& tag_msg){



    int tag_cnt = tag_msg->tag_count;

    if (tag_cnt == 0) return;

    for (int i=0; i<tag_cnt; ++i){
        // ROS_INFO("Found tag  %i (cf: %.3f)", tag_msg->tags[i].id, tag_msg->tags[i].cf);
    }


    for (int i=0; i<tag_cnt; ++i){

        ds_control::Tag tag = tag_msg->tags[i];

        if (tag.id != 1 && tag.id != 12){
            // ROS_INFO("Detected unknown Marker");
            return;
        }


        // detection is too bad
        if (tag.cf < 0.7) continue;

        tf::Transform tag_pose;

        Eigen::Vector3f measurement;

        measurement(0) = tag.zMetric;
        measurement(1) = tag.yMetric;
        measurement(2) = -tag.zRot;

        // ROS_INFO("tag yaw: %f", measurement(2)/M_PI*180);


        if (tag.id == 1){
            // ROS_INFO("Found beta marker at %f %f %.1f", measurement(0),measurement(1),measurement(2)/M_PI*180);
            kalman_filter.correctionStep(measurement, beta_global_pose);
        }
        else{
            //  ROS_INFO("Found zeta marker at %f %f %.1f", measurement(0),measurement(1),measurement(2)/M_PI*180);
            kalman_filter.correctionStep(measurement, zeta_global_pose);
        }


        ekf_marker.addFilterState(kalman_filter.state, kalman_filter.sigma, z);

    }
}



void Ardrone_localizer::navCB(const ds_control::NavdataConstPtr& nav_msg){


    // float bat = nav_msg->batteryPercent;
    // if (bat < 25)
    //  ROS_WARN("low battery: %f %%", bat);


    transform.setOrigin( tf::Vector3(x,y,z) );
    transform.setRotation( tf::Quaternion(nav_msg->rotZ/180*M_PI, nav_msg->rotY/180*M_PI, nav_msg->rotX/180*M_PI) );

    markers.addMarkerPose(tf::StampedTransform(transform, nav_msg->header.stamp, "/world", "/ardrone"));

    if (!got_first_nav){
        got_first_nav = true;
        last_stamp = nav_msg->header.stamp;
        last_yaw = nav_msg->rotZ;
        return;
    }


    double dt_s = (nav_msg->header.stamp-last_stamp).toNSec()/(1000.0*1000.0*1000.0);

    last_stamp = nav_msg->header.stamp;

    float dx =  dt_s*nav_msg->vx/1000; // in m/s
    float dy =  dt_s*nav_msg->vy/1000; // in m/s

    Eigen::Vector3f odometry;
    odometry(0) = dx; // local position update
    odometry(1) = dy; // local position update
    odometry(2) = (nav_msg->rotZ-last_yaw)/180*M_PI; // treat absolute value as incremental update

    last_yaw = nav_msg->rotZ;

    // update pose of robot according to odometry measurement
    // this also increases the uncertainty of the filter
    kalman_filter.predictionStep(odometry);


    z = nav_msg->altd/1000.0;
    ekf_marker.addFilterState(kalman_filter.state, kalman_filter.sigma, z);


    geometry_msgs::Point state;
    state.x = getX();
    state.y = getY();
    state.z = getYaw()/M_PI*180;

    pub_state.publish(state);

    ekf_marker.publish_last_n_states(1);
    markers.publish_markers();

}
