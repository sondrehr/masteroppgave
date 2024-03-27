#ifndef LIE_H
#define LIE_H

#include <ros/ros.h>
#include "sophus/se3.hpp"
#include <iostream>

#include "funcs.h"
#include "classes.h"

Sophus::SE3d rodrigues2sophus(Rodrigues Tvecs);
Sophus::SE3d quaternion2sophus(geometry_msgs::PoseWithCovarianceStamped poseMsg);

Rodrigues sophus2rodrigues(Sophus::SE3d T_cm);
Rodrigues quaternion2rodrigues(geometry_msgs::PoseWithCovarianceStamped poseMsg);

geometry_msgs::PoseWithCovarianceStamped sophus2quaternion(Sophus::SE3d T_cm);
geometry_msgs::PoseWithCovarianceStamped rodrigues2quaternion(Rodrigues Tvecs);



void getMean(Distribution& distribution, int iter);
void getVar(Distribution& distribution);

#endif