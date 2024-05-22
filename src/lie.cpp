#include "lie.h"

Sophus::SE3d rodrigues2sophus(Rodrigues Tvecs)
{
    cv::Mat rotMat(3, 3, CV_64F);
    cv::Rodrigues(Tvecs.rvec, rotMat);

    Eigen::Matrix3d R;    
    R << rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
         rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
         rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2);
    
    Eigen::Vector3d t(Tvecs.tvec[0], Tvecs.tvec[1], Tvecs.tvec[2]);

    Eigen::Matrix4d T_cm;
    T_cm.block<3,3>(0,0) = R;
    T_cm.block<3,1>(0,3) = t;
    T_cm.row(3) << 0, 0, 0, 1;

    return Sophus::SE3d::fitToSE3(T_cm);
}
Sophus::SE3d quaternion2sophus(geometry_msgs::PoseWithCovarianceStamped poseMsg)
{
    Eigen::Quaterniond q(poseMsg.pose.pose.orientation.w, poseMsg.pose.pose.orientation.x, poseMsg.pose.pose.orientation.y, poseMsg.pose.pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();

    Eigen::Vector3d t(poseMsg.pose.pose.position.x, poseMsg.pose.pose.position.y, poseMsg.pose.pose.position.z);

    Eigen::Matrix4d T_cm;
    T_cm.block<3,3>(0,0) = R;
    T_cm.block<3,1>(0,3) = t;
    T_cm.row(3) << 0, 0, 0, 1;

    return Sophus::SE3d::fitToSE3(T_cm);
}

Rodrigues sophus2rodrigues(Sophus::SE3d T_cm)
{
    Eigen::Matrix4d T = T_cm.matrix();
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);

    cv::Vec3d rvec;
    cv::Mat RotMat(3, 3, CV_64F);
    RotMat.at<double>(0, 0) = R(0, 0);
    RotMat.at<double>(0, 1) = R(0, 1);
    RotMat.at<double>(0, 2) = R(0, 2);
    RotMat.at<double>(1, 0) = R(1, 0);
    RotMat.at<double>(1, 1) = R(1, 1);
    RotMat.at<double>(1, 2) = R(1, 2);
    RotMat.at<double>(2, 0) = R(2, 0);
    RotMat.at<double>(2, 1) = R(2, 1);
    RotMat.at<double>(2, 2) = R(2, 2);
    cv::Rodrigues(RotMat, rvec);

    cv::Vec3d tvec(t[0], t[1], t[2]);

    Rodrigues Tvecs;
    Tvecs.rvec = rvec;
    Tvecs.tvec = tvec;

    return Tvecs;
}
Rodrigues quaternion2rodrigues(geometry_msgs::PoseWithCovarianceStamped poseMsg)
{
    Eigen::Quaterniond q(poseMsg.pose.pose.orientation.w, poseMsg.pose.pose.orientation.x, poseMsg.pose.pose.orientation.y, poseMsg.pose.pose.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();

    cv::Vec3d rvec;
    cv::Mat RotMat(3, 3, CV_64F);
    RotMat.at<double>(0, 0) = R(0, 0);
    RotMat.at<double>(0, 1) = R(0, 1);
    RotMat.at<double>(0, 2) = R(0, 2);
    RotMat.at<double>(1, 0) = R(1, 0);
    RotMat.at<double>(1, 1) = R(1, 1);
    RotMat.at<double>(1, 2) = R(1, 2);
    RotMat.at<double>(2, 0) = R(2, 0);
    RotMat.at<double>(2, 1) = R(2, 1);
    RotMat.at<double>(2, 2) = R(2, 2);
    cv::Rodrigues(RotMat, rvec);

    cv::Vec3d tvec(poseMsg.pose.pose.position.x, poseMsg.pose.pose.position.y, poseMsg.pose.pose.position.z);

    Rodrigues Tvecs;
    Tvecs.rvec = rvec;
    Tvecs.tvec = tvec;

    return Tvecs;
}

geometry_msgs::PoseWithCovarianceStamped rodrigues2quaternion(Rodrigues Tvecs)
{
    Sophus::SE3d T_cm_SE3 = rodrigues2sophus(Tvecs);

    geometry_msgs::PoseWithCovarianceStamped poseMsg;
    poseMsg.pose.pose.position.x = T_cm_SE3.translation()[0];
    poseMsg.pose.pose.position.y = T_cm_SE3.translation()[1];
    poseMsg.pose.pose.position.z = T_cm_SE3.translation()[2];
    poseMsg.pose.pose.orientation.w = T_cm_SE3.unit_quaternion().w();
    poseMsg.pose.pose.orientation.x = T_cm_SE3.unit_quaternion().x();
    poseMsg.pose.pose.orientation.y = T_cm_SE3.unit_quaternion().y();
    poseMsg.pose.pose.orientation.z = T_cm_SE3.unit_quaternion().z();

    return poseMsg;
}
geometry_msgs::PoseWithCovarianceStamped sophus2quaternion(Sophus::SE3d T_cm)
{
    geometry_msgs::PoseWithCovarianceStamped poseMsg;
    poseMsg.pose.pose.position.x = T_cm.translation()[0];
    poseMsg.pose.pose.position.y = T_cm.translation()[1];
    poseMsg.pose.pose.position.z = T_cm.translation()[2];
    poseMsg.pose.pose.orientation.w = T_cm.unit_quaternion().w();
    poseMsg.pose.pose.orientation.x = T_cm.unit_quaternion().x();
    poseMsg.pose.pose.orientation.y = T_cm.unit_quaternion().y();
    poseMsg.pose.pose.orientation.z = T_cm.unit_quaternion().z();

    return poseMsg;
}



void getMean(Distribution &distribution, int iter)
{
    distribution.mean = distribution.posesSE3.at(0);
    double w = 1.0d/distribution.posesSE3.size();

    for (int i = 0; i < iter; i++)
    {
        // Get update vector
        Sophus::SE3d::Tangent update = Sophus::SE3d::Tangent::Zero();
        for (size_t j = 0; j < distribution.posesSE3.size(); j++)
        {
            update += w * (distribution.mean.inverse() * distribution.posesSE3.at(j)).log();
        }

        // Update pose
        Sophus::SE3d newMean = distribution.mean * Sophus::SE3d::exp(update);

        // Break condition
        Sophus::SE3d::Tangent diff = (newMean.inverse() * distribution.mean).log();

        distribution.mean = newMean;
        if (Sophus::squaredNorm<Sophus::SE3d::Tangent>(diff) < Sophus::Constants<double>::epsilon()) {return;}
    }
}
void getVar(Distribution &distribution)
{
    int nPoses = distribution.posesSE3.size();
    distribution.var = Sophus::SE3d::Tangent::Zero();

    for (int i = 0; i < nPoses; i++)
    {
        Sophus::SE3d::Tangent error = (distribution.mean.inverse() * distribution.posesSE3.at(i)).log();

        distribution.var += error.cwiseProduct(error);

        //Only calulates the variance part of the covariance matrix since this is only used for comparison
        //If want full covariance matrix, change the dimensions of distribution.var & uncomment the following line
        
        //distribution.var += error * error.transpose();
    }
    
    distribution.var /= nPoses-1;
    distribution.stdDev = distribution.var.cwiseSqrt();
}
