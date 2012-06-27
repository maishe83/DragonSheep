
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef NODE_HPP_
#define NODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "ds_control/Navdata.h"
#include <ds_hog/hog_paramsConfig.h>
#include <opencv2/objdetect/objdetect.hpp>


const double d2r = 3.14159265/*358979323*/ / 180.0;

/*****************************************************************************
 ** Class
 *****************************************************************************/


class QNode{

	public:
		QNode(int argc, char** argv);
        ~QNode();

	private:
	    void incoming_image(const sensor_msgs::ImageConstPtr& msg);
        void navCB(const ds_control::NavdataConstPtr& nav_msg);
        void setParameter(ds_hog::hog_paramsConfig &config, uint32_t level);

        double dyn_val;
        bool rollcomp;

        image_transport::Subscriber sub_image;
        image_transport::Publisher pub_image;
        ros::Subscriber sub_nav; //drone sensors
        ros::NodeHandle* n;
        image_transport::ImageTransport* im_transport;
        ds_control::Navdata nav_last;

        cv::HOGDescriptor hog;

};

#endif
