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
    arucoDetector cam("src/detector/params.yaml", 0.096, true);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/cam0/cam0", 1, &arucoDetector::imageCallback, &cam);

    ros::spin();
    cv::destroyWindow("view");

    return 0;
}
