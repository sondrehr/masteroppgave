#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

#include "funcs.h"
#include <std_msgs/String.h>

#include "classes.h"

#include <iostream>

class Rectify
{
public:
    ros::NodeHandle nh;
    image_transport::CameraPublisher pub;
    image_transport::Subscriber sub;
    sensor_msgs::CameraInfo cam;

    void callback(const sensor_msgs::ImageConstPtr& msg)    
    {
        cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat outputImage;

        std::string filename;
        nh.getParam("/rectify/cameraParams", filename);
        Camera camera(filename);

        bool invalidPixels = false;
        nh.getParam("/rectify/invalidPixels", invalidPixels);

        if (!invalidPixels)
        {
            cv::undistort(inputImage, outputImage, camera.K, camera.D);

            cam.K = {camera.K.at<double>(0,0), 0, camera.K.at<double>(0,2), 0, camera.K.at<double>(1,1), camera.K.at<double>(1,2), 0, 0, 1};
            cam.P = {camera.K.at<double>(0,0), 0, camera.K.at<double>(0,2), 0, 0, camera.K.at<double>(1,1), camera.K.at<double>(1,2), 0, 0, 0, 1, 0};
            cam.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            cam.D = {camera.D.at<double>(0), camera.D.at<double>(1), camera.D.at<double>(2), camera.D.at<double>(3), 0};
        }
        else
        {
            cv::Mat K = cv::getOptimalNewCameraMatrix(camera.K, camera.D, inputImage.size(), 1);
            cv::Mat map1, map2;
            cv::initUndistortRectifyMap(camera.K, camera.D, cv::Mat(), K, inputImage.size(), CV_32FC1, map1, map2);
            cv::remap(inputImage, outputImage, map1, map2, cv::INTER_LINEAR);

            cam.K = {K.at<double>(0,0), 0, K.at<double>(0,2), 0, K.at<double>(1,1), K.at<double>(1,2), 0, 0, 1};
            cam.P = {K.at<double>(0,0), 0, K.at<double>(0,2), 0, 0, K.at<double>(1,1), K.at<double>(1,2), 0, 0, 0, 1, 0};
            cam.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            cam.D = {camera.D.at<double>(0), camera.D.at<double>(1), camera.D.at<double>(2), camera.D.at<double>(3), 0};
        }

        sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();

        cam.header.stamp = msg->header.stamp;
        cam.header.frame_id = "cam0";
        
    

        pub.publish(*imgMsg, cam, cam.header.stamp);
    }
    Rectify(){
        std::string subTopic;
        std::string pubTopic;

        nh.getParam("/rectify/subTopic", subTopic);
        nh.getParam("/rectify/pubTopic", pubTopic);

        pub = image_transport::ImageTransport(nh).advertiseCamera(pubTopic, 1);
        
        image_transport::ImageTransport it(nh);
        sub = it.subscribe(subTopic, 1, &Rectify::callback, this);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");

    Rectify rectify;

    ros::spin();
    return 0;
}
