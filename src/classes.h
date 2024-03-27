#ifndef CLASSES_H
#define CLASSES_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "structs.h"
#include "sophus/se3.hpp"

#include "precision_landing/myAprilTagDetectionArray.h"
#include "precision_landing/myAprilTagDetection.h"



class Camera
{
public:
    cv::Mat K;
    cv::Mat D;

    Camera(std::string filename)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        fs["Camera_Matrix"] >> K;
        fs["Distortion_Coefficients"] >> D;
    }
};


class Detector : public Camera
{
public:
    ros::Publisher                       pubDist;
    image_transport::Publisher           pubImg;
    image_transport::Subscriber          subImg;

    bool estimateState;
    bool publishImage;
    bool publishDistribution;
    bool publishPose;
    bool publishTrajectory;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    std::vector<geometry_msgs::PoseWithCovarianceStamped> estimatedPoses;
    std::deque<std::vector<geometry_msgs::PoseWithCovarianceStamped>> posesCov;
    Distribution distribution;

    float beta;
    float markerLength;

    Detector(std::string filename) : Camera(filename){};
};



// Aruco
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class arucoDetector : public Detector
{
public:
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    arucoDetector(ros::NodeHandle &nh);
};

// April
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class aprilDetector : public Detector
{
public:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const precision_landing::myAprilTagDetectionArrayConstPtr& detections);
    aprilDetector(ros::NodeHandle &nh);
};


#endif