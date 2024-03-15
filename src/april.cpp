#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include "functions.h"
#include "lie.h"
#include "classes.h"
#include "structs.h"


    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    //aprilDetector cam("../Desktop/catkin_ws/src/subscriber/params.yaml", 0.095, true);
    aprilDetector cam("../Desktop/catkin_ws/src/subscriber/params.yaml", 0.095, true);

    message_filters::Subscriber<sensor_msgs::Image> sub1(nh, "/cam0/cam0", 1);
    message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> sub2(nh, "/tag_detections", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, apriltag_ros::AprilTagDetectionArray> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind(&aprilDetector::imageCallback, &cam, _1, _2));
    

    //ros::Subscriber sub = nh.subscribe("/tag_detections", 1, &aprilDetector::imageCallback, &cam);
 
    ros::spin();
    cv::destroyWindow("view");

    return 0;
}