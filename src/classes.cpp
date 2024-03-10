#include "classes.h"
#include "functions.h"
#include "lie.h"



void arucoDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat outputImage = inputImage.clone();

    detectMarkersAruco(this, inputImage);
    estimatePoseAruco(this); 
    drawMarkersAruco(this, outputImage);

    if (dist) {getDistribution(this, 50, inputImage);}
    
    cv::imshow("view", outputImage);
    cv::waitKey(1);
}

