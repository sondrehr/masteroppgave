#ifndef STRUCTS2_H
#define STRUCTS2_H

#include <opencv2/opencv.hpp>
#include "sophus/se3.hpp"

struct empty {};

struct Tvec
{
    cv::Vec3d rvec;
    cv::Vec3d tvec;
};

struct Distribution
{
    std::vector<Sophus::SE3d> poses;    
    Sophus::SE3d mean;                      
    Sophus::SE3d::Tangent var; 
};

#endif