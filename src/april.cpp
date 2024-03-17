#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <precision_landing/myAprilTagDetectionArray.h>

#include "classes.h"

    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    aprilDetector cam("../Desktop/catkin_ws/src/precision_landing/params.yaml", 0.09, true);

    message_filters::Subscriber<sensor_msgs::Image> sub1(nh, "/cam0/cam0", 1);
    message_filters::Subscriber<precision_landing::myAprilTagDetectionArray> sub2(nh, "/tag_detections", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, precision_landing::myAprilTagDetectionArray> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind(&aprilDetector::imageCallback, &cam, _1, _2));
     
    ros::spin();
    cv::destroyWindow("view");

    return 0;
}