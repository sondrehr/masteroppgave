#ifndef LIE_H
#define LIE_H

#include <ros/ros.h>
#include "sophus/se3.hpp"
#include <iostream>

#include "functions.h"
#include "classes.h"

Sophus::SE3d tvec2sophus(Tvec Tvecs);
Tvec sophus2tvec(Sophus::SE3d T_cm);

void getMean(Distribution& distribution, int iter);
void getVar(Distribution& distribution);

//Change name of func below
void getDistribution(arucoDetector* detector, int nPoses, cv::Mat& inputImage);

#endif