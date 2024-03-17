#ifndef CLASSES_H
#define CLASSES_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "structs.h"
//#include "apriltag_ros/AprilTagDetectionArray.h"
//#include "apriltag_ros/AprilTagDetection.h"


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
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    std::deque<std::vector<geometry_msgs::PoseWithCovarianceStamped>> posesCov;
    Distribution distribution;

    Detector(std::string filename) : Camera(filename){};
};



// Aruco
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class arucoDetector : public Detector
{
public:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    
    bool dist;
    float markerLength;  
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    arucoDetector(std::string filename, float markerLength, bool dist) : Detector(filename)
    {
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
        this->markerLength = markerLength;
        this->dist = dist;
    }
};

// April
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class aprilDetector : public Detector
{
public:
    bool dist;
    float markerLength;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const precision_landing::myAprilTagDetectionArrayConstPtr& detections);
    aprilDetector(std::string filename, float markerLength, bool dist) : Detector(filename)
    {
        this->dist = dist;
        this->markerLength = markerLength;
    }
};


#endif