#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "sophus/se3.hpp"

#include "classes.h"


void detectMarkersAruco(arucoDetector* detector, cv::Mat& inputImage);

void drawMarkersAruco(arucoDetector* detector, cv::Mat& outputImage);

void estimatePoseAruco(arucoDetector* detector);



#endif