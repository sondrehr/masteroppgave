#include <precision_landing/distribution.h>
#include <precision_landing/trajectoryInterpolated.h>
#include <precision_landing/estimate.h>
#include "lie.h"
#include <iostream>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <tf2_ros/transform_broadcaster.h>
#include "sophus/se3.hpp"


template <class T>
void estimateState(T* detector)
{
    if (detector->markerIds.size() != 1) {return;}

    precision_landing::estimate srv;
    srv.request.poseIn = detector->posesCov.back().at(0);

    if(detector->servEstimate.call(srv))
    {
        detector->posesCov.back().at(0) = srv.response.poseOut;
    }
    else
    {
        ROS_ERROR("Failed to call service estimateState");
    }     
}

template <class T>
void publishTrajectory(T* detector)
{
    if (detector->markerIds.size() != 1) {return;}     /// Maybe change to multiple markers??? use:    for (size_t i = 0; i < detector->markerIds.size(); i++){trajectory.joint_names.push_back("tag_" + std::to_string(detector->markerIds.at(i)) + "_joint)

    //Move the pose to the platform coordinate system
    Sophus::SE3d T_cam_tag = quaternion2sophus(detector->posesCov.back().at(0));
    
    Eigen::Matrix4d T_plat_cam = Eigen::Matrix4d::Identity();
    T_plat_cam.block<3,3>(0,0) = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_plat_cam.block<3,1>(0,3) = Eigen::Vector3d(0.099, 0, 0);

    Eigen::Matrix4d T_tag_drone = Eigen::Matrix4d::Identity();
    T_tag_drone.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    Sophus::SE3d dronePose = Sophus::SE3d::fitToSE3(T_plat_cam)*T_cam_tag*Sophus::SE3d::fitToSE3(T_tag_drone);
    
    //Define the interpolation points
    cv::Point3f closestPoint(0, 0, dronePose.translation()[2]);
    cv::Point3f dronePos(dronePose.translation()[0], dronePose.translation()[1], dronePose.translation()[2]);
    cv::Point3f endPos(0, 0, 0);

    double distance = cv::norm(closestPoint - dronePos);
    uint16_t nJointLateral = distance*detector->nJointDensity;
    uint16_t nJointVertical = dronePos.z*detector->nJointDensity;
    uint16_t nJointDirect = cv::norm(endPos - dronePos)*detector->nJointDensity;
    
    //Define the poses at the interpolation points
    Eigen::Vector3d t_closest(0, 0, closestPoint.z);
    Eigen::Matrix4d T_closestPoseMatrix = Eigen::Matrix4d::Identity();
    T_closestPoseMatrix.block<3,1>(0,3) = t_closest;

    Eigen::Matrix4d T_endPoseMatrix = Eigen::Matrix4d::Identity();

    Sophus::SE3d closestPose = Sophus::SE3d::fitToSE3(T_closestPoseMatrix);
    Sophus::SE3d endPose = Sophus::SE3d::fitToSE3(T_endPoseMatrix);

    //Get the yaw of the drone
    Eigen::Vector3d x_axis_drone = dronePose.so3().unit_quaternion().toRotationMatrix().col(0);           
    double yaw = atan2(x_axis_drone[1], x_axis_drone[0]);

    //Objects to be published
    trajectory_msgs::MultiDOFJointTrajectory trajectory;
    trajectory.header.stamp = detector->posesCov.back().at(0).header.stamp;
    trajectory.header.frame_id = "platform";

    geometry_msgs::PoseArray trajectory_interpolated;
    trajectory_interpolated.header.frame_id = "platform";

    //Interpolate the trajectory
    for(uint16_t i = 0; i < nJointDirect; i++)
    {
        double time = (float)i/(float)nJointDirect;

        //Rotation
        Eigen::Matrix3d R;
        if (detector->rotInt=="yaw")
        {
            Eigen::Matrix3d R_yaw;
            R_yaw << std::cos((1-time)*yaw), -std::sin((1-time)*yaw), 0,
                    std::sin((1-time)*yaw),  std::cos((1-time)*yaw), 0,
                    0, 0, 1;

            R = R_yaw;
        }
        else if (detector->rotInt=="rotMat")
        {
            Sophus::SO3d R_drone = dronePose.so3();
            Sophus::SO3d R_end = endPose.so3();

            R = (R_drone*Sophus::SO3d::exp((R_drone.inverse()*R_end).log()*time)).matrix();
        }
        
        //Translation
        Eigen::Vector3d t;
        if (detector->pathInt=="straight")
        {
            t[0] = dronePos.x + time*(endPos.x - dronePos.x);
            t[1] = dronePos.y + time*(endPos.y - dronePos.y);
            t[2] = dronePos.z + time*(endPos.z - dronePos.z);
        } 
        else if (detector->pathInt=="bezier")
        {
            Eigen::Vector3d p0(dronePos.x, dronePos.y, dronePos.z);
            Eigen::Vector3d p1(closestPoint.x, closestPoint.y, closestPoint.z);
            Eigen::Vector3d p2(endPos.x, endPos.y, endPos.z);

            Eigen::Vector3d p01 = p0 + time*(p1 - p0);
            Eigen::Vector3d p12 = p1 + time*(p2 - p1);
            t = p01 + time*(p12 - p01);
        }
        else if (detector->pathInt=="rth")
        {
            if ((float)nJointLateral/(nJointLateral+nJointVertical) > (float)i/nJointDirect)
            {
                double timeLat = ((float)i/nJointDirect)/((float)nJointLateral/(nJointLateral+nJointVertical));
                t[0] = dronePos.x + timeLat*(closestPoint.x - dronePos.x);
                t[1] = dronePos.y + timeLat*(closestPoint.y - dronePos.y);
                t[2] = dronePos.z;
            }
            else
            {
                double timeVert = ((float)i/nJointDirect - (float)nJointLateral/(nJointLateral+nJointVertical))/((float)nJointVertical/(nJointLateral+nJointVertical));
                t[0] = 0;
                t[1] = 0;
                t[2] = closestPoint.z + timeVert*(endPos.z - closestPoint.z);
            }
        }

        //Create interpolated pose
        Eigen::Matrix4d T_interpolatedPoseMatrix = Eigen::Matrix4d::Identity();
        T_interpolatedPoseMatrix.block<3,3>(0,0) = R;
        T_interpolatedPoseMatrix.block<3,1>(0,3) = t;
        Sophus::SE3d interpolatedPose = Sophus::SE3d::fitToSE3(T_interpolatedPoseMatrix);

        if(detector->pathInt=="full") {interpolatedPose = dronePose*Sophus::SE3d::exp((dronePose.inverse()*endPose).log()*time);}
        else if(detector->pathInt=="full_bezier")
        {
            Sophus::SE3d p0 = dronePose;
            Sophus::SE3d p1 = closestPose;
            Sophus::SE3d p2 = endPose;

            Sophus::SE3d p01 = p0*Sophus::SE3d::exp((p0.inverse()*p1).log()*time);
            Sophus::SE3d p12 = p1*Sophus::SE3d::exp((p1.inverse()*p2).log()*time);
            interpolatedPose = p01*Sophus::SE3d::exp((p01.inverse()*p12).log()*time);
        }

        //Create message
        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        point.time_from_start = ros::Duration((float)i*1.5);
        point.transforms.resize(1);
        //point.velocities.resize(1);
        //point.accelerations.resize(1);
        point.transforms[0].translation.x = interpolatedPose.translation()[0];
        point.transforms[0].translation.y = interpolatedPose.translation()[1];
        point.transforms[0].translation.z = interpolatedPose.translation()[2];
        point.transforms[0].rotation.w = interpolatedPose.unit_quaternion().w();
        point.transforms[0].rotation.x = interpolatedPose.unit_quaternion().x();
        point.transforms[0].rotation.y = interpolatedPose.unit_quaternion().y();
        point.transforms[0].rotation.z = interpolatedPose.unit_quaternion().z();
        trajectory.points.push_back(point);

        geometry_msgs::Pose pose;
        pose.position.x = point.transforms[0].translation.x;
        pose.position.y = point.transforms[0].translation.y;
        pose.position.z = point.transforms[0].translation.z;
        pose.orientation.w = point.transforms[0].rotation.w;
        pose.orientation.x = point.transforms[0].rotation.x;
        pose.orientation.y = point.transforms[0].rotation.y;
        pose.orientation.z = point.transforms[0].rotation.z;
        trajectory_interpolated.poses.push_back(pose);
    }
    detector->pubTraj.publish(trajectory);
    detector->pubTrajInterpolated.publish(trajectory_interpolated);
}


template <class T>
void publishTF(T* detector)
{
    for (size_t i = 0; i < detector->posesCov.back().size(); i++)
    {   
        static tf2_ros::TransformBroadcaster pubTF;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = detector->posesCov.back().at(i).header.stamp;
        transformStamped.header.frame_id = "cam";
        transformStamped.child_frame_id = "aprilTag_" + std::to_string(detector->markerIds.at(i));
        transformStamped.transform.translation.x = detector->posesCov.back().at(i).pose.pose.position.x;
        transformStamped.transform.translation.y = detector->posesCov.back().at(i).pose.pose.position.y;
        transformStamped.transform.translation.z = detector->posesCov.back().at(i).pose.pose.position.z;
        transformStamped.transform.rotation = detector->posesCov.back().at(i).pose.pose.orientation;
        
        pubTF.sendTransform(transformStamped);
    }
}


template <class T>
void publishDistribution(T* detector, int nPoses)
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
        //msg.var = {detector->distribution.var[0], detector->distribution.var[1], detector->distribution.var[2], detector->distribution.var[3], detector->distribution.var[4], detector->distribution.var[5]};
        msg.stdDev = {detector->distribution.stdDev[0], detector->distribution.stdDev[1], detector->distribution.stdDev[2], detector->distribution.stdDev[3], detector->distribution.stdDev[4], detector->distribution.stdDev[5]};

        detector->pubDist.publish(msg);
    }
}


template <class T>
void publishImage(T* detector, cv::Mat& outputImage)
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

    if (detector->publishBeta){std::cout << "Beta: " << detector->beta/100 << std::endl;}
}



