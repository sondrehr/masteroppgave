#include "classes.h"
#include "funcs.h"

#include <precision_landing/myAprilTagDetectionArray.h>
#include <precision_landing/distribution.h>

#include <tf2_msgs/TFMessage.h>



void arucoDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;

    detectMarkersAruco(this, inputImage);
    getPoseAruco(this);
    if (estimateState)       {stateEstimation(this);}
    if (publishImage)        {drawMarkers(this, inputImage);}
    if (publishDistribution) {getDistribution(this, 100);}
    if (publishPose)         {publishTF(this);}
    if (publishTrajectory)   {getTrajectory(this);}
}


void aprilDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg, const precision_landing::myAprilTagDetectionArrayConstPtr& detections)
{
    cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;

    getMarkersApril(this, detections);
    getPoseApril(this, detections);
    if (estimateState)       {stateEstimation(this);}
    if (publishImage)        {drawMarkers(this, inputImage);}
    if (publishDistribution) {getDistribution(this, 100);}
    if (publishPose)         {publishTF(this);}
    if (publishTrajectory)   {getTrajectory(this);}
}


arucoDetector::arucoDetector(ros::NodeHandle &nh) : Detector(getParams<std::string>(nh, "/aruco/cameraParams"))
{
    subImg = image_transport::ImageTransport(nh).subscribe("/cam0_rect/cam0", 1, &arucoDetector::imageCallback, this);

    pubImg = image_transport::ImageTransport(nh).advertise("/aruco/image", 1);
    pubDist = nh.advertise<precision_landing::distribution>("/aruco/dist", 1);

    markerLength        = getParams<float>(nh, "/aruco/markerSize");
    beta                = 0.0;

    estimateState       = getParams<bool>(nh, "/aruco/estimateState");
    publishImage        = getParams<bool>(nh, "/aruco/publishImage");
    publishDistribution = getParams<bool>(nh, "/aruco/publishDistribution");
    publishPose         = getParams<bool>(nh, "/aruco/publishPose");
    publishTrajectory   = getParams<bool>(nh, "/aruco/publishTrajectory");
    
    if      (getParams<std::string>(nh, "aruco/dictionary") == "4X4") {dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);}
    else if (getParams<std::string>(nh, "aruco/dictionary") == "5X5") {dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);}
    else if (getParams<std::string>(nh, "aruco/dictionary") == "6X6") {dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);}   
}
aprilDetector::aprilDetector(ros::NodeHandle &nh) : Detector(getParams<std::string>(nh, "/april/cameraParams"))
{
    pubImg = image_transport::ImageTransport(nh).advertise("/april/image", 1);
    pubDist = nh.advertise<precision_landing::distribution>("/april/dist", 1);

    markerLength        = getParams<float>(nh, "/april/markerSize");
    beta                = 0.0;

    estimateState       = getParams<bool>(nh, "/april/estimateState");
    publishImage        = getParams<bool>(nh, "/april/publishImage");
    publishDistribution = getParams<bool>(nh, "/april/publishDistribution");
    publishPose         = getParams<bool>(nh, "/april/publishPose");
    publishTrajectory   = getParams<bool>(nh, "/april/publishTrajectory");
}
