#include <precision_landing/distribution.h>
#include <iostream>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sophus/se3.hpp"

template <class T>
void stateEstimation(T* detector)
{
    //Implement kalman filter and IPDA
    std::vector<geometry_msgs::PoseWithCovarianceStamped> prevPoses = detector->estimatedPoses;
}

template <class T>
void getTrajectory(T* detector)
{
    //Implement trajectory estimation
}

template <class T>
void publishTF(T* detector)
{
    for (size_t i = 0; i < detector->posesCov.back().size(); i++)
    {   
        static tf2_ros::TransformBroadcaster pubTF;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = detector->posesCov.back().at(i).header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "tag_" + std::to_string(detector->markerIds.at(i));
        transformStamped.transform.translation.x = detector->posesCov.back().at(i).pose.pose.position.x;
        transformStamped.transform.translation.y = detector->posesCov.back().at(i).pose.pose.position.y;
        transformStamped.transform.translation.z = detector->posesCov.back().at(i).pose.pose.position.z;
        transformStamped.transform.rotation = detector->posesCov.back().at(i).pose.pose.orientation;
        
        pubTF.sendTransform(transformStamped);
    }
}

template <class T>
void getDistribution(T* detector, int nPoses)
{   
    if (detector->markerIds.size() != 1) {return;}

    Sophus::SE3d thisPose = quaternion2sophus(detector->posesCov.back().at(0));
    detector->distribution.posesSE3.push_back(thisPose);

    if (detector->distribution.posesSE3.size() == nPoses)
    {
        getMean(detector->distribution, 50);
        getVar(detector->distribution);
        detector->distribution.posesSE3.pop_front();


        precision_landing::distribution msg;
        Rodrigues Tvec = sophus2rodrigues(detector->distribution.mean);

        msg.mean = {Tvec.tvec[0], Tvec.tvec[1], Tvec.tvec[2], Tvec.rvec[0], Tvec.rvec[1], Tvec.rvec[2]};
        msg.var = {detector->distribution.var[0], detector->distribution.var[1], detector->distribution.var[2], detector->distribution.var[3], detector->distribution.var[4], detector->distribution.var[5]};
        msg.stdDev = {detector->distribution.stdDev[0], detector->distribution.stdDev[1], detector->distribution.stdDev[2], detector->distribution.stdDev[3], detector->distribution.stdDev[4], detector->distribution.stdDev[5]};

        detector->pubDist.publish(msg);
    }
}

template <class T>
void drawMarkers(T* detector, cv::Mat& outputImage)
{
    cv::aruco::drawDetectedMarkers(outputImage, detector->markerCorners, detector->markerIds);
    for (size_t i = 0; i < detector->posesCov.back().size(); i++)
    {
        Rodrigues Tvec = quaternion2rodrigues(detector->posesCov.back().at(i));
        cv::aruco::drawAxis(outputImage, detector->K, cv::Mat::zeros(5, 1, CV_32F), Tvec.rvec, Tvec.tvec, detector->markerLength*1.5);
    }

    detector->pubImg.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg());
}

template <typename T>
T getParams(ros::NodeHandle &nh, std::string param)
{
    T value;
    nh.getParam(param, value);
    return value;
}

template <class T>
void getBeta(T* detector)
{
    if (detector->markerIds.size() != 0) {detector->beta++;}
    if (detector->posesCov.size() == 100 && detector->posesCov.front().size() != 0) {detector->beta--;}

    std::cout << "Beta: " << detector->beta/100 << std::endl;
}



