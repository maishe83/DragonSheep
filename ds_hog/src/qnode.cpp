/**
 * @file /eros_qtalker/src/qnode.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 25/02/2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include "../include/qnode.hpp"


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv_modules.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <sstream>
#include <exception>
#include "ds_control/Navdata.h"

#include <ds_hog/hog_paramsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/bind.hpp>
namespace enc = sensor_msgs::image_encodings;



cv::Mat rotateImage(const cv::Mat& source, double angle){
    cv::Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, angle, 1.0);
    cv::Mat dst;
    cv::warpAffine(source, dst, rot_mat, source.size());
    return dst;
}


/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv) {
    ros::init(argc, argv, "ds_hog");
    n = new ros::NodeHandle();
    im_transport = new image_transport::ImageTransport(*n);

    sub_image = im_transport->subscribe("/ardrone/image_raw", 1, &QNode::incoming_image, this);
    pub_image = im_transport->advertise("/ds_hog/image", 1);

    sub_nav   = n->subscribe("/ardrone/navdata", 1, &QNode::navCB, this);



    // dynamic_reconfigure to change the pid-params on the fly
    dynamic_reconfigure::Server<ds_hog::hog_paramsConfig> srv;
    dynamic_reconfigure::Server<ds_hog::hog_paramsConfig>::CallbackType f;
    f = boost::bind(&QNode::setParameter, this,  _1, _2);
    srv.setCallback(f);

    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());


    ROS_INFO_STREAM("Starting <ds_hog node>");
    ros::spin();
}


QNode::~QNode() {
    //n->shutdown();
    //wait();
    ROS_INFO_STREAM("Shutting <ds_hog node> down");
}

void QNode::setParameter(ds_hog::hog_paramsConfig &config, uint32_t level){
    dyn_val = config.factorVal;
    rollcomp = config.rollComp;
}

void QNode::incoming_image(const sensor_msgs::ImageConstPtr& msg) {
    // Convert ROS images to OpenCV images
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg,enc::MONO8);// enc::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM(e.what());
        return;
    }
    //cv_ptr->image;

    //TODO Operate on CV Image
    const double rotX = nav_last.rotX;

    if (rollcomp)
        cv_ptr->image =  rotateImage(cv_ptr->image, rotX*-1);

    std::vector<cv::Rect> found;
    hog.detectMultiScale(cv_ptr->image, found);
    for(unsigned i = 0; i < found.size(); i++) {
        cv::Rect r = found[i];
        rectangle(cv_ptr->image, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
    }
    cv::waitKey(5);


    //TODO Publish image
    pub_image.publish(cv_ptr->toImageMsg());
}




void QNode::navCB(const ds_control::NavdataConstPtr& nav_msg) {
    nav_last = *nav_msg;
//    //Skip first msg as we have no prev msg
//    if (prev_stamp < 0) {
//        prev_stamp = nav_msg->header.stamp.toSec();
//        init_stamp = prev_stamp;
//        return;
//    }

//    //Compute delta time and update last known time
//    const double curr_stamp = nav_msg->header.stamp.toSec();
//    const double delta_time = curr_stamp - prev_stamp;
//    prev_stamp = curr_stamp;

//    //Print rosmsg
//    //ROS_INFO("vX: %fmm/s vY: %fmm/s Alt: %dmm Yaw: %f° dT: %f", nav_msg->vx, nav_msg->vy, nav_msg->altd, nav_msg->rotZ, delta_time);


//    //Compute rotations by convering to radians
//    const double rotX = nav_msg->rotX * d2r;
//    const double rotY = nav_msg->rotY * d2r;
//    const double rotZ = nav_msg->rotZ * d2r;

//    Q_EMIT(rotmsg(nav_msg->rotX, nav_msg->rotY, nav_msg->rotZ));

//    //Set computed rotations as current state (via convering to quaternions)
//    tf::Quaternion q;
//    q.setEulerZYX(rotZ, rotY, rotX);
//    current_state.setRotation(q);

//    //Convert given speeds to distances by converting from mm to m and multi by delta time
//    const double dx = nav_msg->vx / 1000.0 * delta_time;
//    const double dy = nav_msg->vy / 1000.0 * delta_time;

//    //Z is given to us as a distance in mm, convert to meters
//    const double z = static_cast<double> (nav_msg->altd) / 1000.0;

//    //tf::Vector3 vchange(dx, dy, 0.0); //ROS fuerte
//    tf::Vector3 vchange(dx, dy, 0.0);
//    //tf::Vector3 new_state(0.0, 0.0, 0.0); //ROS fuerte
//    tf::Vector3 new_state(0.0, 0.0, 0.0);

//    //Build a rotation matrix from the 3 given rotations
//    //tf::Matrix3x3 rotmatrix; //ROS fuerte
//    tf::Matrix3x3 rotmatrix;
//    rotmatrix.setEulerZYX(rotZ, rotY, rotX);

//    //Apply it to the vector to get new state
//    vchange = rotmatrix * vchange;
//    new_state = vchange + current_state.getOrigin();
//    new_state.setZ(z);

//    //Sum distance
//    total_distance += new_state.distance(current_state.getOrigin());
//    //ROS_INFO_STREAM("Distance Travelled so far: " << total_distance);

//    //Average Altitude
//    average_alt.push_back(z);
//    //ROS_INFO_STREAM("Average Altitude: "<< (std::accumulate(average_alt.begin(), average_alt.end(), 0.0) / average_alt.size()) << "m");

//    current_state.setOrigin(new_state);

//    //Set the altitude
//    current_state.getOrigin().setZ(z);



//    //add current state to markers
//    addMarkerPose(tf::StampedTransform(current_state, nav_msg->header.stamp, "/world", "/ardrone"));

//    //Publish
//    publish_markers();

}


