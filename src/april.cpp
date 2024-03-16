#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <subscriber/myAprilTagDetectionArray.h>

#include "classes.h"

    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    aprilDetector cam("../Desktop/catkin_ws/src/subscriber/params.yaml", 0.09, true);

    message_filters::Subscriber<sensor_msgs::Image> sub1(nh, "/cam0/cam0", 1);
    message_filters::Subscriber<subscriber::myAprilTagDetectionArray> sub2(nh, "/tag_detections", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, subscriber::myAprilTagDetectionArray> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind(&aprilDetector::imageCallback, &cam, _1, _2));
     
    ros::spin();
    cv::destroyWindow("view");

    return 0;
}