#include "classes.h"
#include "functions.h"


void arucoDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat outputImage = inputImage.clone();

    detectMarkersAruco(this, inputImage);
    getPoseAruco(this); 
    drawMarkersAruco(this, outputImage);
    if (dist) {getDistributionAruco(this, 50);}
    
    cv::imshow("view", outputImage);
    cv::waitKey(1);
}


void aprilDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg, const precision_landing::myAprilTagDetectionArrayConstPtr& detections)
{
    cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat outputImage = inputImage.clone();

    getMarkersApril(this, detections);
    getPoseApril(this, detections);
    drawMarkersApril(this, outputImage);
    if (dist) {getDistributionApril(this, 50);}

    cv::imshow("view", outputImage);
    cv::waitKey(1);
}
