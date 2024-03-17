#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <ros/ros.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "sophus/se3.hpp"
#include "classes.h"


void detectMarkersAruco(arucoDetector* detector, cv::Mat& inputImage);
void getPoseAruco(arucoDetector* detector);
void drawMarkersAruco(arucoDetector* detector, cv::Mat& outputImage);
void getDistributionAruco(arucoDetector* detector, int nPoses);

void getMarkersApril(aprilDetector* detector, const precision_landing::myAprilTagDetectionArrayConstPtr& detections);
void getPoseApril(aprilDetector* detector, const precision_landing::myAprilTagDetectionArrayConstPtr& detections);
void drawMarkersApril(aprilDetector* detector, cv::Mat& outputImage);
void getDistributionApril(aprilDetector* detector, int nPoses);


#endif