#ifndef STRUCTS2_H
#define STRUCTS2_H

#include <opencv2/opencv.hpp>
#include "sophus/se3.hpp"

struct Rodrigues
{
    cv::Vec3d rvec;
    cv::Vec3d tvec;
};

struct Distribution
{
    std::deque<Sophus::SE3d> posesSE3;    
    
    Sophus::SE3d mean;                      
    Sophus::SE3d::Tangent var; 
    Sophus::SE3d::Tangent stdDev;
};

#endif