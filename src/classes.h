#ifndef CLASSES_H
#define CLASSES_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "structs.h"


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
    std::vector<std::vector<cv::Point2f>> markerCorners;        ///???
    std::vector<Tvec> Tvecs;

    ///// add to below later
    std::vector<geometry_msgs::PoseWithCovarianceStamped> poses;

    Detector(std::string filename) : Camera(filename){};
};



// Aruco
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <bool T>
class arucoDetector : public Detector
{
public:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    float markerLength;

    bool dist;
    //std::conditional<T, Distribution, empty> distribution;
    Distribution distribution;  
    
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
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    aprilDetector(std::string filename, float markerLength, bool dist) : Detector(filename){};
};


#endif