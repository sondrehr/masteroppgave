#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include "functions.h"
#include "lie.h"
#include "classes.h"
#include "structs.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;

    cv::namedWindow("view");
 
 
    ros::spin();
    cv::destroyWindow("view");

    return 0;
}