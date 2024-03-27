#include "funcs.h"
#include "lie.h"
#include "classes.h"

#include <precision_landing/distribution.h>


void detectMarkersAruco(arucoDetector* detector, cv::Mat& outputImage)
{
    cv::aruco::detectMarkers(outputImage, detector->dictionary, detector->markerCorners, detector->markerIds);
    
    getBeta(detector);

    if (detector->posesCov.size() == 100) {detector->posesCov.pop_front();}

    detector->posesCov.push_back(std::vector<geometry_msgs::PoseWithCovarianceStamped>());
    detector->posesCov.back().resize(detector->markerIds.size());
}
void getPoseAruco(arucoDetector* detector)
{
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-detector->markerLength/2, detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(detector->markerLength/2, detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(detector->markerLength/2, -detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-detector->markerLength/2, -detector->markerLength/2, 0);

    for (size_t i = 0; i < detector->markerIds.size(); i++)
    {   
        Rodrigues Tvec;
        cv::solvePnP(objPoints, detector->markerCorners.at(i), detector->K, cv::Mat::zeros(5, 1, CV_32F), Tvec.rvec, Tvec.tvec);

        cv::Mat J;
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objPoints, Tvec.rvec, Tvec.tvec, detector->K, cv::Mat::zeros(5, 1, CV_32F), projectedPoints, J);
        cv::Mat sigma = cv::Mat(J.t() * J, cv::Rect(0, 0, 6, 6)).inv();         // 6x6 to only get the cov matrix of R and t

        detector->posesCov.back().at(i) = rodrigues2quaternion(Tvec);
        detector->posesCov.back().at(i).header.stamp = ros::Time::now();
        detector->posesCov.back().at(i).pose.covariance = {sigma.at<double>(0, 0), sigma.at<double>(0, 1), sigma.at<double>(0, 2), sigma.at<double>(0, 3), sigma.at<double>(0, 4), sigma.at<double>(0, 5),
                                                          sigma.at<double>(1, 0), sigma.at<double>(1, 1), sigma.at<double>(1, 2), sigma.at<double>(1, 3), sigma.at<double>(1, 4), sigma.at<double>(1, 5),
                                                          sigma.at<double>(2, 0), sigma.at<double>(2, 1), sigma.at<double>(2, 2), sigma.at<double>(2, 3), sigma.at<double>(2, 4), sigma.at<double>(2, 5),
                                                          sigma.at<double>(3, 0), sigma.at<double>(3, 1), sigma.at<double>(3, 2), sigma.at<double>(3, 3), sigma.at<double>(3, 4), sigma.at<double>(3, 5),
                                                          sigma.at<double>(4, 0), sigma.at<double>(4, 1), sigma.at<double>(4, 2), sigma.at<double>(4, 3), sigma.at<double>(4, 4), sigma.at<double>(4, 5),
                                                          sigma.at<double>(5, 0), sigma.at<double>(5, 1), sigma.at<double>(5, 2), sigma.at<double>(5, 3), sigma.at<double>(5, 4), sigma.at<double>(5, 5)};
    }
}


void getMarkersApril(aprilDetector* detector, const precision_landing::myAprilTagDetectionArrayConstPtr& detections)
{
    detector->markerIds.resize(detections->detections.size());
    for (size_t i = 0; i < detections->detections.size(); i++)
    {
        detector->markerIds.at(i) = detections->detections.at(i).id[0];
    }  

    /// maybe delete
    detector->markerCorners.resize(detections->detections.size());
    for (size_t i = 0; i < detections->detections.size(); i++)
    {
        Rodrigues Tvec = quaternion2rodrigues(detections->detections.at(i).pose);
        cv::Mat objPoints(4, 1, CV_32FC3);
        std::vector<cv::Point2f> projectedPoints;
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-detector->markerLength/2, detector->markerLength/2, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(detector->markerLength/2, detector->markerLength/2, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(detector->markerLength/2, -detector->markerLength/2, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-detector->markerLength/2, -detector->markerLength/2, 0);

        cv::projectPoints(objPoints, Tvec.rvec, Tvec.tvec, detector->K, cv::Mat::zeros(5, 1, CV_32F), projectedPoints);

        detector->markerCorners.at(i).resize(4);
        for (size_t j = 0; j < 4; j++)
        {
            detector->markerCorners.at(i).at(j).x = projectedPoints.at(j).x;
            detector->markerCorners.at(i).at(j).y = projectedPoints.at(j).y;
        }
    }

    getBeta(detector);

    if (detector->posesCov.size() == 100) {detector->posesCov.pop_front();}

    detector->posesCov.push_back(std::vector<geometry_msgs::PoseWithCovarianceStamped>());
    detector->posesCov.back().resize(detections->detections.size()); 
}
void getPoseApril(aprilDetector* detector, const precision_landing::myAprilTagDetectionArrayConstPtr& detections)
{
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-detector->markerLength/2, detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(detector->markerLength/2, detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(detector->markerLength/2, -detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-detector->markerLength/2, -detector->markerLength/2, 0);

    for (size_t i = 0; i < detections->detections.size(); i++)
    {
        geometry_msgs::PoseWithCovarianceStamped poseMsg = detections->detections.at(i).pose;
        Rodrigues Tvec = quaternion2rodrigues(poseMsg);

        cv::Mat J;
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objPoints, Tvec.rvec, Tvec.tvec, detector->K, cv::Mat::zeros(5, 1, CV_32F), projectedPoints, J);
        cv::Mat sigma = cv::Mat(J.t() * J, cv::Rect(0, 0, 6, 6)).inv();         // 6x6 to only get the cov matrix of R and t

        detector->posesCov.back().at(i).pose.pose.position.x = detections->detections.at(i).pose.pose.pose.position.x;
        detector->posesCov.back().at(i).pose.pose.position.y = detections->detections.at(i).pose.pose.pose.position.y;
        detector->posesCov.back().at(i).pose.pose.position.z = detections->detections.at(i).pose.pose.pose.position.z;
        detector->posesCov.back().at(i).pose.pose.orientation.x = detections->detections.at(i).pose.pose.pose.orientation.x;
        detector->posesCov.back().at(i).pose.pose.orientation.y = detections->detections.at(i).pose.pose.pose.orientation.y;
        detector->posesCov.back().at(i).pose.pose.orientation.z = detections->detections.at(i).pose.pose.pose.orientation.z;
        detector->posesCov.back().at(i).pose.pose.orientation.w = detections->detections.at(i).pose.pose.pose.orientation.w;
        detector->posesCov.back().at(i).header.stamp = detections->header.stamp;
        detector->posesCov.back().at(i).pose.covariance = {sigma.at<double>(0, 0), sigma.at<double>(0, 1), sigma.at<double>(0, 2), sigma.at<double>(0, 3), sigma.at<double>(0, 4), sigma.at<double>(0, 5),
                                                          sigma.at<double>(1, 0), sigma.at<double>(1, 1), sigma.at<double>(1, 2), sigma.at<double>(1, 3), sigma.at<double>(1, 4), sigma.at<double>(1, 5),
                                                          sigma.at<double>(2, 0), sigma.at<double>(2, 1), sigma.at<double>(2, 2), sigma.at<double>(2, 3), sigma.at<double>(2, 4), sigma.at<double>(2, 5),
                                                          sigma.at<double>(3, 0), sigma.at<double>(3, 1), sigma.at<double>(3, 2), sigma.at<double>(3, 3), sigma.at<double>(3, 4), sigma.at<double>(3, 5),
                                                          sigma.at<double>(4, 0), sigma.at<double>(4, 1), sigma.at<double>(4, 2), sigma.at<double>(4, 3), sigma.at<double>(4, 4), sigma.at<double>(4, 5),
                                                          sigma.at<double>(5, 0), sigma.at<double>(5, 1), sigma.at<double>(5, 2), sigma.at<double>(5, 3), sigma.at<double>(5, 4), sigma.at<double>(5, 5)};
    }
}


