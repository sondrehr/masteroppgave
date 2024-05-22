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
    ros::NodeHandle                      nh;

    image_transport::Subscriber          subImg;
    image_transport::Publisher           pubImg;
    ros::Publisher                       pubDist;
    ros::Publisher                       pubTraj;
    ros::Publisher                       pubTrajInterpolated;


    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    std::vector<geometry_msgs::PoseWithCovarianceStamped> estimatedPoses;                        // Only for 10 markers. Use the marker id to access the pose
    std::deque<std::vector<geometry_msgs::PoseWithCovarianceStamped>> posesCov;
    Distribution distribution;

    float beta = 0.0;
    float markerLength;
    bool publishBeta;
    int nJointDensity;
    int nPoses;

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