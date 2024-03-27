#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

#include "funcs.h"
#include <std_msgs/String.h>

#include "classes.h"

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

        //Camera camera("../params.yaml");
        Camera camera("../Desktop/catkin_ws/src/precision_landing/params.yaml");
        cv::undistort(inputImage, outputImage, camera.K, camera.D);
        //cv::remap(inputImage, outputImage, camera.map1, camera.map2, cv::INTER_LINEAR);

        sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();


        cam.header.stamp = msg->header.stamp;
        cam.header.frame_id = "cam0";
        cam.K = {camera.K.at<double>(0,0), 0, camera.K.at<double>(0,2), 0, camera.K.at<double>(1,1), camera.K.at<double>(1,2), 0, 0, 1};
        cam.P = {camera.K.at<double>(0,0), 0, camera.K.at<double>(0,2), 0, 0, camera.K.at<double>(1,1), camera.K.at<double>(1,2), 0, 0, 0, 1, 0};
        cam.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        cam.D = {camera.D.at<double>(0), camera.D.at<double>(1), camera.D.at<double>(2), camera.D.at<double>(3), 0};
    

        pub.publish(*imgMsg, cam, cam.header.stamp);
    }
    Rectify(){
        pub = image_transport::ImageTransport(nh).advertiseCamera("/cam0_rect/cam0", 1);
        
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("/cam0/cam0", 1, &Rectify::callback, this);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");

    Rectify rectify;

    ros::spin();
    return 0;
}
