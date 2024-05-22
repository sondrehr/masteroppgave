#include <precision_landing/distribution.h>
#include <precision_landing/trajectoryInterpolated.h>
#include "lie.h"
#include <iostream>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <tf2_ros/transform_broadcaster.h>
#include "sophus/se3.hpp"


template <class T>
void estimateState(T* detector)
{
    //Implement kalman filter and IPDA??
    std::vector<geometry_msgs::PoseWithCovarianceStamped> prevPoses = detector->estimatedPoses;
}

template <class T>
void publishTrajectory(T* detector)
{
    if (detector->markerIds.size() != 1) {return;}     /// Maybe change to multiple markers??? use:    for (size_t i = 0; i < detector->markerIds.size(); i++){trajectory.joint_names.push_back("tag_" + std::to_string(detector->markerIds.at(i)) + "_joint)

    cv::Point3f closestPoint, dronePos, endPos;
    closestPoint.x = 0;
    closestPoint.y = 0;
    closestPoint.z = detector->posesCov.back().at(0).pose.pose.position.z;

    dronePos.x = detector->posesCov.back().at(0).pose.pose.position.x;
    dronePos.y = detector->posesCov.back().at(0).pose.pose.position.y;
    dronePos.z = detector->posesCov.back().at(0).pose.pose.position.z;

    endPos.x = 0;
    endPos.y = 0;
    endPos.z = 0;

    uint16_t nJointLateral = cv::norm(closestPoint - dronePos)*detector->nJointDensity;
    uint16_t nJointVertical = dronePos.z*detector->nJointDensity;
    uint16_t nJointDirect = cv::norm(endPos - dronePos)*detector->nJointDensity;

    //Test
    /////////////////////////////////////////////////
    Eigen::Matrix3d R;
    Eigen::Vector3d t_closest;
    Eigen::Vector3d t_end;
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    t_closest << 0, 0, detector->posesCov.back().at(0).pose.pose.position.z;
    t_end << 0, 0, 0;
    
    Eigen::Matrix4d T_closestPoseMatrix;
    T_closestPoseMatrix.block<3,3>(0,0) = R;
    T_closestPoseMatrix.block<3,1>(0,3) = t_closest;
    T_closestPoseMatrix.row(3) << 0, 0, 0, 1;

    Eigen::Matrix4d T_endPoseMatrix;
    T_endPoseMatrix.block<3,3>(0,0) = R;
    T_endPoseMatrix.block<3,1>(0,3) = t_end;
    T_endPoseMatrix.row(3) << 0, 0, 0, 1;

    Sophus::SE3d dronePose = quaternion2sophus(detector->posesCov.back().at(0));
    Sophus::SE3d closestPose = Sophus::SE3d::fitToSE3(T_closestPoseMatrix);
    Sophus::SE3d endPose = Sophus::SE3d::fitToSE3(T_endPoseMatrix);
    /////////////////////////////////////////////////        

    //Objects to be published
    trajectory_msgs::MultiDOFJointTrajectory trajectory;
    trajectory.header.stamp = detector->posesCov.back().at(0).header.stamp;
    trajectory.header.frame_id = "platform";

    geometry_msgs::PoseArray trajectory_interpolated;
    trajectory_interpolated.header.frame_id = "platform";

    std::string path = "straight";

    if (path=="straight")
    {
        for(uint16_t i = 0; i < nJointDirect; i++)
        {
            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::Pose pose;

            Sophus::SE3d interpolatedPose;

            /////////////////////////////////////////////////

            point.time_from_start = ros::Duration((float)i*1.5);
            point.transforms.resize(1);
            point.velocities.resize(1);
            point.accelerations.resize(1);

            //t
            /*
            point.transforms[0].translation.x = (1-(float)i/(float)nJointDirect)*dronePos.x;
            point.transforms[0].translation.y = (1-(float)i/(float)nJointDirect)*dronePos.y;
            point.transforms[0].translation.z = (1-(float)i/(float)nJointDirect)*dronePos.z;

            pose.position.x = (1-(float)i/(float)nJointDirect)*dronePos.x;
            pose.position.y = (1-(float)i/(float)nJointDirect)*dronePos.y;
            pose.position.z = (1-(float)i/(float)nJointDirect)*dronePos.z;
            */

            //R
            Eigen::Vector3d x_axis_drone = dronePose.so3().unit_quaternion().toRotationMatrix().col(0);
            x_axis_drone[2] = 0;
            Eigen::Vector3d x_axis_platform;
            x_axis_platform << 1, 0, 0;

            double yaw = atan2(x_axis_drone[1], x_axis_drone[0]);
            
            Eigen::Matrix3d R_yaw;
            Eigen::Vector3d t;

            R_yaw << std::cos((1-(float)i/(float)nJointLateral)*yaw), -std::sin((1-(float)i/(float)nJointLateral)*yaw), 0,
                     std::sin((1-(float)i/(float)nJointLateral)*yaw),  std::cos((1-(float)i/(float)nJointLateral)*yaw), 0,
                     0, 0, 1;

            t[0] = (1-(float)i/(float)nJointDirect)*dronePos.x;
            t[1] = (1-(float)i/(float)nJointDirect)*dronePos.y;
            t[2] = (1-(float)i/(float)nJointDirect)*dronePos.z;

            Eigen::Matrix4d T_interpolatedPoseMatrix;
            T_interpolatedPoseMatrix.block<3,3>(0,0) = R_yaw;
            T_interpolatedPoseMatrix.block<3,1>(0,3) = t;
            T_interpolatedPoseMatrix.row(3) << 0, 0, 0, 1;
            
            interpolatedPose = Sophus::SE3d::fitToSE3(T_interpolatedPoseMatrix);

            geometry_msgs::PoseWithCovarianceStamped pointPose;
            pointPose.pose.pose.position.x = interpolatedPose.translation()[0];
            pointPose.pose.pose.position.y = interpolatedPose.translation()[1];
            pointPose.pose.pose.position.z = interpolatedPose.translation()[2];
            pointPose.pose.pose.orientation.w = interpolatedPose.unit_quaternion().w();
            pointPose.pose.pose.orientation.x = interpolatedPose.unit_quaternion().x();
            pointPose.pose.pose.orientation.y = interpolatedPose.unit_quaternion().y();
            pointPose.pose.pose.orientation.z = interpolatedPose.unit_quaternion().z();
            
            point.transforms[0].translation.x = pointPose.pose.pose.position.x;
            point.transforms[0].translation.y = pointPose.pose.pose.position.y;
            point.transforms[0].translation.z = pointPose.pose.pose.position.z;
            point.transforms[0].rotation = pointPose.pose.pose.orientation;
            trajectory.points.push_back(point);

            pose.position.x = pointPose.pose.pose.position.x;
            pose.position.y = pointPose.pose.pose.position.y;
            pose.position.z = pointPose.pose.pose.position.z;
            pose.orientation = pointPose.pose.pose.orientation;
            trajectory_interpolated.poses.push_back(pose);
        }
    }


    //


    /*

    if (distance > 0.05)
    {
        for(uint16_t i = 0; i < nJointLateral; i++)
        {
            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::Pose pose;

            /////////////////////////////////////////////////

            point.time_from_start = ros::Duration((float)i*1.5);
            point.transforms.resize(1);
            point.velocities.resize(1);
            point.accelerations.resize(1);

            point.transforms[0].translation.x = dronePos.x + (closestPoint.x - dronePos.x)*i/nJointLateral;
            point.transforms[0].translation.y = dronePos.y + (closestPoint.y - dronePos.y)*i/nJointLateral;
            point.transforms[0].translation.z = dronePos.z;
            //trajectory.points.push_back(point);

            pose.position.x = dronePos.x + (closestPoint.x - dronePos.x)*i/nJointLateral;
            pose.position.y = dronePos.y + (closestPoint.y - dronePos.y)*i/nJointLateral;
            pose.position.z = dronePos.z;
            //trajectory_interpolated.poses.push_back(pose);

            //Test
            /////////////////////////////////////////////////
            Sophus::SE3d::Tangent update = (dronePose.inverse()*closestPose).log();
            Sophus::SE3d interpolatedPose = dronePose*Sophus::SE3d::exp(update*(float)i/(float)nJointLateral);


    /*
            //Interpolation yaw
            /////////////////////////////////////////////////
            Eigen::Vector3d x_axis_drone = dronePose.so3().unit_quaternion().toRotationMatrix().col(0);
            x_axis_drone[2] = 0;
            Eigen::Vector3d x_axis_platform;
            x_axis_platform << 1, 0, 0;

            double yaw = atan2(x_axis_drone[1], x_axis_drone[0]);
            
            Eigen::Matrix3d R_yaw;
            Eigen::Vector3d t;

            R_yaw << std::cos((1-(float)i/(float)nJointLateral)*yaw), -std::sin((1-(float)i/(float)nJointLateral)*yaw), 0,
                     std::sin((1-(float)i/(float)nJointLateral)*yaw),  std::cos((1-(float)i/(float)nJointLateral)*yaw), 0,
                     0, 0, 1;

            t[0] = dronePos.x + (closestPoint.x - dronePos.x)*i/nJointLateral;
            t[1] = dronePos.y + (closestPoint.y - dronePos.y)*i/nJointLateral;
            t[2] = dronePos.z;

            interpolatedPose = Sophus::SE3d(R_yaw, t);
            /////////////////////////////////////////////////
    */

    /*
            //Interpolation Lie R,t corrected
            /////////////////////////////////////////////////        
            Sophus::SO3d R = interpolatedPose.so3();
            Eigen::Vector3d t = interpolatedPose.translation();

            Sophus::SO3d::Tangent rvec = R.log();

            rvec[0] = 0;
            rvec[1] = 0;
            t[2] = dronePos.z;

            R = Sophus::SO3d::exp(rvec);

            interpolatedPose = Sophus::SE3d(R, t);
            /////////////////////////////////////////////////
    */


    /*
            //Interpolation Lie R, regular t
            /////////////////////////////////////////////////        
            Sophus::SO3d R = interpolatedPose.so3();
            Eigen::Vector3d t = interpolatedPose.translation();

            Sophus::SO3d::Tangent rvec = R.log();

            rvec[0] = 0;
            rvec[1] = 0;

            R = Sophus::SO3d::exp(rvec);

            t[0] = dronePos.x + (closestPoint.x - dronePos.x)*i/nJointLateral;
            t[1] = dronePos.y + (closestPoint.y - dronePos.y)*i/nJointLateral;
            t[2] = dronePos.z;

            interpolatedPose = Sophus::SE3d(R, t);
    
            /////////////////////////////////////////////////
    */

    /*
            geometry_msgs::PoseWithCovarianceStamped pointPose;
            pointPose.pose.pose.position.x = interpolatedPose.translation()[0];
            pointPose.pose.pose.position.y = interpolatedPose.translation()[1];
            pointPose.pose.pose.position.z = interpolatedPose.translation()[2];
            pointPose.pose.pose.orientation.w = interpolatedPose.unit_quaternion().w();
            pointPose.pose.pose.orientation.x = interpolatedPose.unit_quaternion().x();
            pointPose.pose.pose.orientation.y = interpolatedPose.unit_quaternion().y();
            pointPose.pose.pose.orientation.z = interpolatedPose.unit_quaternion().z();
            
            point.transforms[0].translation.x = pointPose.pose.pose.position.x;
            point.transforms[0].translation.y = pointPose.pose.pose.position.y;
            point.transforms[0].translation.z = pointPose.pose.pose.position.z;
            point.transforms[0].rotation = pointPose.pose.pose.orientation;
            trajectory.points.push_back(point);

            pose.position.x = pointPose.pose.pose.position.x;
            pose.position.y = pointPose.pose.pose.position.y;
            pose.position.z = pointPose.pose.pose.position.z;
            pose.orientation = pointPose.pose.pose.orientation;
            trajectory_interpolated.poses.push_back(pose);
            
            /////////////////////////////////////////////////
        }
    }

    for(uint16_t i = 0; i <= nJointVertical; i++)
    {
        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::Pose pose;
        if (distance > 0.05) {point.time_from_start = ros::Duration((float)nJointLateral*1.5 + (float)i*1.5);}
        else                 {point.time_from_start = ros::Duration((float)i*1.5);}

        point.transforms.resize(1);
        point.velocities.resize(1);
        point.accelerations.resize(1);

        point.transforms[0].translation.x = 0;
        point.transforms[0].translation.y = 0;
        point.transforms[0].translation.z = (1-(float)i/(float)nJointVertical)*dronePos.z;

        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = (1-(float)i/(float)nJointVertical)*dronePos.z;

        trajectory_interpolated.poses.push_back(pose);
    }

    */

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



