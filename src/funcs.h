#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <ros/ros.h>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "sophus/se3.hpp"
#include "classes.h"
#include "lie.h"
#include "funcs_template.tcc"

template <typename T> T getParams(ros::NodeHandle &nh, std::string param);
template <class T> void getBeta(T* detector);
template <class T> void getDistribution(T* detector, int nPoses);
template <class T> void drawMarkers(T* detector, cv::Mat& outputImage);
template <class T> void publishTF(T* detector);

void detectMarkersAruco(arucoDetector* detector, cv::Mat& inputImage);
void getPoseAruco(arucoDetector* detector);

void getMarkersApril(aprilDetector* detector, const precision_landing::myAprilTagDetectionArrayConstPtr& detections);
void getPoseApril(aprilDetector* detector, const precision_landing::myAprilTagDetectionArrayConstPtr& detections);

#endif