#include "classes.h"
#include "funcs.h"

#include <precision_landing/myAprilTagDetectionArray.h>
#include <precision_landing/distribution.h>
#include <precision_landing/trajectoryInterpolated.h>
#include <precision_landing/estimate.h>

void arucoDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;

    detectMarkersAruco(this, inputImage);
    getPoseAruco(this);
    if (getParams<bool>(this->nh, "/aruco/estimateState"))       {estimateState(this);}
    if (getParams<bool>(this->nh, "/aruco/publishImage"))        {publishImage(this, inputImage);}
    if (getParams<bool>(this->nh, "/aruco/publishDistribution")) {publishDistribution(this, 100);}                  //only for one marker
    if (getParams<bool>(this->nh, "/aruco/publishTF"))           {publishTF(this);}
    if (getParams<bool>(this->nh, "/aruco/publishTrajectory"))   {publishTrajectory(this);}                         //only for one marker
}

void aprilDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg, const precision_landing::myAprilTagDetectionArrayConstPtr& detections)
{
    cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;

    getMarkersApril(this, detections);
    getPoseApril(this, detections);
    if (getParams<bool>(this->nh, "/april/estimateState"))       {estimateState(this);}   
    if (getParams<bool>(this->nh, "/april/publishImage"))        {publishImage(this, inputImage);}
    if (getParams<bool>(this->nh, "/april/publishDistribution")) {publishDistribution(this, 100);}                  //only for one marker
    if (getParams<bool>(this->nh, "/april/publishTF"))           {publishTF(this);}
    if (getParams<bool>(this->nh, "/april/publishTrajectory"))   {publishTrajectory(this);}                         //only for one marker
}



arucoDetector::arucoDetector(ros::NodeHandle &nh) : Detector(getParams<std::string>(nh, "/aruco/cameraParams"))
{
    this->nh                = nh;
    subImg                  = image_transport::ImageTransport(nh).subscribe("/cam0_rect/cam0", 1, &arucoDetector::imageCallback, this);
    pubImg                  = image_transport::ImageTransport(nh).advertise("/aruco/image", 1);
    pubDist                 = nh.advertise<precision_landing::distribution>("/aruco/dist", 1);
    pubTraj                 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/aruco/trajectory", 1);
    pubTrajInterpolated     = nh.advertise<geometry_msgs::PoseArray>("/aruco/trajectoryInterpolated", 1);
    servEstimate            = nh.serviceClient<precision_landing::estimate>("/estimate_state");

    markerLength     = getParams<float>(nh, "/aruco/markerSize");
    publishBeta      = getParams<bool>(nh, "/aruco/publishBeta");
    pathInt          = getParams<std::string>(nh, "/aruco/pathInt");
    rotInt           = getParams<std::string>(nh, "/aruco/rotInt");
    nJointDensity    = getParams<int>(nh, "/aruco/nJointDensity");
    nPoses           = getParams<int>(nh, "/aruco/nPoses");
    estimatedPoses.resize(nPoses);
    
    if      (getParams<std::string>(nh, "aruco/dictionary") == "4X4") {dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);}
    else if (getParams<std::string>(nh, "aruco/dictionary") == "5X5") {dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);}
    else if (getParams<std::string>(nh, "aruco/dictionary") == "6X6") {dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);}   
}

aprilDetector::aprilDetector(ros::NodeHandle &nh) : Detector(getParams<std::string>(nh, "/april/cameraParams"))
{
    this->nh                = nh;
    pubImg                  = image_transport::ImageTransport(nh).advertise("/april/image", 1);
    pubDist                 = nh.advertise<precision_landing::distribution>("/april/dist", 1);
    pubTraj                 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/april/trajectory", 1);
    pubTrajInterpolated     = nh.advertise<geometry_msgs::PoseArray>("/april/trajectoryInterpolated", 1);
    servEstimate            = nh.serviceClient<precision_landing::estimate>("/estimate_state");

    markerLength     = getParams<float>(nh, "/april/markerSize");
    publishBeta      = getParams<bool>(nh, "/april/publishBeta");
    pathInt          = getParams<std::string>(nh, "/april/pathInt");
    rotInt           = getParams<std::string>(nh, "/april/rotInt");
    nJointDensity    = getParams<int>(nh, "/april/nJointDensity");
    nPoses           = getParams<int>(nh, "/april/nPoses");
    estimatedPoses.resize(nPoses);
}
