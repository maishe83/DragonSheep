/*
 * Visual Navigation for Flying Robots:
 *
 * Sheet 3
 *
 *  Created on: May 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <ds_control/pdi_paramsConfig.h>


#include "ds_control/Navdata.h"
#include "ds_control/Tag.h"
#include "ds_control/Tags.h"
#include "ds_control/marker.h"
#include "ds_control/EKF.h"
#include "ds_control/pid_controller.h"
#include "ds_control/Ardrone_localizer.h"



using namespace std;
using namespace Eigen;



struct Ardrone_controller {
    ros::Publisher pub_vel;
    ros::NodeHandle* nh_;

    ros::Publisher pub_cmd_marker;

    // new command
    geometry_msgs::Twist twist;

    Ardrone_localizer* localizer;

    PID_Controller pid_x;
    PID_Controller pid_y;
    PID_Controller pid_z;
    PID_Controller pid_yaw;

    float x_goal, y_goal, height_goal, yaw_goal;

    double radius, speed, altitude, heading;
    bool fly_circle, active;

    void setParameter(ds_control::pdi_paramsConfig &config){
        pid_x.c_proportional = pid_y.c_proportional = pid_z.c_proportional = config.c_prop_trans;
        pid_x.c_integral = pid_y.c_integral = pid_z.c_integral = config.c_int_trans;
        pid_x.c_derivative = pid_y.c_derivative = pid_z.c_derivative = config.c_deriv_trans;

        pid_yaw.c_proportional = config.c_prop_yaw;
        pid_yaw.c_integral = config.c_int_yaw;
        pid_yaw.c_derivative = config.c_deriv_yaw;

        radius = config.Radius;
        speed = config.Speed;
        fly_circle = config.FlyCircle;
        active = config.ControllerOn;
        altitude = config.Altitude;
        heading = config.Heading;
    }


    void init(){
        nh_ = new ros::NodeHandle();
        pub_vel = nh_->advertise<geometry_msgs::Twist>("/cmd_vel",1);
        pub_cmd_marker = nh_->advertise<visualization_msgs::Marker>( "cmd_marker", 10);

        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
        setGoalPose(0,0,1,0); // hover 1m above origin
    }

    // use this to set a new goal pose for the controller
    void setGoalPose(float x, float y, float height, float yaw){
        x_goal = x;
        y_goal = y;
        height_goal=height;
        yaw_goal=yaw;
    }


    // control in xy and yaw
    void sendNewCommand(){

        ros::Time now = ros::Time::now();

        float yaw_current = localizer->getYaw();

        //error = goal - current [all in world coordinates]
        float ex2 = x_goal-localizer->getX();
        float ey2 = y_goal-localizer->getY();
        float eyaw = yaw_goal - yaw_current;

        //convert to local frame and normalise angle [Local frame]
        float ex =  cos(yaw_current)*ex2 + sin(yaw_current)*ey2;
        float ey = -sin(yaw_current)*ex2 + cos(yaw_current)*ey2;
        eyaw = atan2(sin(eyaw),cos(eyaw));

        // create and send message
        twist.linear.x = pid_x.getCommand(ex,now);
        twist.linear.y = pid_y.getCommand(ey,now);
        twist.angular.z = pid_yaw.getCommand(eyaw,now);
        pub_vel.publish(twist);

        sendCmdMarker();
    }


    void sendCmdMarker(){

        visualization_msgs::Marker marker;

        marker.header.frame_id = "/world";

        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "cmd_marker_ns";
        marker.header.stamp = ros::Time::now();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05; // width of line in m
        marker.scale.y = 0.1; // width of line in m
        geometry_msgs::Point p,q;
        p.x = localizer->getX();
        p.y = localizer->getY();
        p.z = localizer->getHeight();

        float length = 0.5;

        marker.color.a = 1.0;
        marker.color.g = 1.0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.points.clear();
        marker.id = 0;

        float yaw = -localizer->getYaw();
        q.x = p.x + length*(cos(yaw)*twist.linear.x+sin(yaw)*twist.linear.y);
        q.y = p.y + length*(-sin(yaw)*twist.linear.x+cos(yaw)*twist.linear.y);
        q.z = p.z + length*twist.linear.z;

        marker.points.push_back(p);
        marker.points.push_back(q);
        pub_cmd_marker.publish(marker);


        // show rotation: attach arrow to end of pose-marker-arrow and rotate by 90deg
        // length is proportional to the rotation speed
        marker.points.clear();
        p.x = localizer->getX() + cos(yaw)*0.5; // 0.5: length of pose-marker-arrow
        p.y = localizer->getY() - sin(yaw)*0.5;
        q.x = p.x  - 1*(cos(yaw+M_PI/2)*twist.angular.z); // 1: arbitrary constant (angular.z is in rad, marker in meter)
        q.y = p.y  - 1*(-sin(yaw+M_PI/2)*twist.angular.z);
        q.z = p.z + length*twist.linear.z;
        marker.points.push_back(p);
        marker.points.push_back(q);
        marker.id = 1;
        marker.color.g = 0; marker.color.b = 1.0;

        pub_cmd_marker.publish(marker);
    }

};




Ardrone_controller controller;

void dynReconfCB(ds_control::pdi_paramsConfig &config, uint32_t level){
    controller.setParameter(config);
}


float incr = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_visualization");

    Ardrone_localizer localizer;

    controller.init();
    controller.localizer = &localizer;


    tf::Transform tag_pose_zeta, tag_pose_beta, pose_goal;
    tag_pose_zeta.setOrigin(tf::Vector3(localizer.zeta_global_pose.x(),
                                        localizer.zeta_global_pose.y(),
                                        localizer.zeta_global_pose.z()));
    tag_pose_beta.setOrigin(tf::Vector3(localizer.beta_global_pose.x(),
                                        localizer.beta_global_pose.y(),
                                        localizer.beta_global_pose.z()));
    pose_goal.setOrigin(tf::Vector3(0,0,1));
    tag_pose_zeta.setRotation( tf::Quaternion(0,0,0));
    tag_pose_beta.setRotation( tf::Quaternion(0,0,0));
    pose_goal.setRotation(tf::Quaternion(0,0,0));

    tf::TransformBroadcaster br;


    // dynamic_reconfigure to change the pid-params on the fly
    dynamic_reconfigure::Server<ds_control::pdi_paramsConfig> srv;
    dynamic_reconfigure::Server<ds_control::pdi_paramsConfig>::CallbackType f;
    f = boost::bind(&dynReconfCB, _1, _2);
    srv.setCallback(f);

    ros::Rate r(30);
    float gx,gy,gz,gyaw;
    while (localizer.nh_.ok()){
        ros::spinOnce();

        if (controller.fly_circle){
            // continuously change goal pose
            incr += 0.05 * controller.speed;
        }
        gz = controller.altitude;
        //fly a circle

        gx = controller.radius * sin(incr);
        gy = controller.radius * cos(incr);
        gyaw = atan2(gy,gx)+ M_PI/180*(-controller.heading-90);

        // update tf of new goal and set new goal for quadcopter




        controller.setGoalPose(gx,gy,gz,gyaw);

        if (controller.active){
            controller.sendNewCommand();
        }

        pose_goal.setOrigin(tf::Vector3(gx,gy,gz));
        pose_goal.setRotation(tf::Quaternion(0, 0, gyaw));
        //br.sendTransform(tf::StampedTransform(tag_pose_zeta, ros::Time::now(), "/world", "/zeta_marker"));
        //br.sendTransform(tf::StampedTransform(tag_pose_beta, ros::Time::now(), "/world", "/beta_marker"));
        br.sendTransform(tf::StampedTransform(pose_goal, ros::Time::now(), "/world", "/goal_marker"));
        r.sleep();
    }

    return 0;
}
