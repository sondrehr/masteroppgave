#include "lie.h"

Sophus::SE3d tvec2sophus(Tvec Tvecs)
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
Tvec sophus2tvec(Sophus::SE3d T_cm)
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

    Tvec Tvecs;
    Tvecs.rvec = rvec;
    Tvecs.tvec = tvec;

    return Tvecs;
}

void getMean(Distribution &distribution, int iter)
{
    distribution.mean = distribution.poses.at(0);
    double w = 1.0d/distribution.poses.size();

    for (int i = 0; i < iter; i++)
    {
        // Get update vector
        Sophus::SE3d::Tangent update = Sophus::SE3d::Tangent::Zero();
        for (size_t j = 0; j < distribution.poses.size(); j++)
        {
            update += w * (distribution.mean.inverse() * distribution.poses.at(j)).log();
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
    int nPoses = distribution.poses.size();
    distribution.var = Sophus::SE3d::Tangent::Zero();

    for (int i = 0; i < nPoses; i++)
    {
        Sophus::SE3d::Tangent error = (distribution.mean.inverse() * distribution.poses.at(i)).log();
        distribution.var += error.cwiseProduct(error);
    }
    
    distribution.var /= nPoses;
}

void getDistribution(arucoDetector* detector, int nPoses, cv::Mat& inputImage)
{
    if (detector->markerIds.size() != 1) {return;}

    Sophus::SE3d thisPose = tvec2sophus(detector->Tvecs.at(0));
    detector->distribution.poses.push_back(thisPose);

    if (detector->distribution.poses.size() == nPoses)
    {
        getMean(detector->distribution, 50);
        getVar(detector->distribution);

        std::cout << "Mean: \n" << detector->distribution.mean.matrix() << std::endl;
        std::cout << "\nVariance: \n" << detector->distribution.var << std::endl;
    }
}





















            /*
            std::vector<vecs> avgVecs = {sophus2vecs(detector->avgPose)};
            detector->Vecs = avgVecs;

            cv::Mat avgImage = inputImage.clone();
            drawMarkersAruco(detector, avgImage);
            cv::imwrite("avgImage.png", avgImage);
            */