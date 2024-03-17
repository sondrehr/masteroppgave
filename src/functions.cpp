#include "functions.h"
#include "lie.h"
#include "classes.h"


void detectMarkersAruco(arucoDetector* detector, cv::Mat& inputImage)
{
    cv::aruco::detectMarkers(inputImage, detector->dictionary, detector->markerCorners, detector->markerIds);
    
    if (detector->posesCov.size() == 50) {detector->posesCov.pop_front();}

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
        cv::solvePnP(objPoints, detector->markerCorners.at(i), detector->K, detector->D, Tvec.rvec, Tvec.tvec);
        //cv::solvePnPRefineLM(objPoints, detector->markerCorners.at(i), detector->K, detector->D, Tvec.rvec, Tvec.tvec);
        //cv::solvePnPRefineVVS(objPoints, detector->markerCorners.at(i), detector->K, detector->D, Tvec.rvec, Tvec.tvec);

        cv::Mat J;
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objPoints, Tvec.rvec, Tvec.tvec, detector->K, detector->D, projectedPoints, J);
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
void drawMarkersAruco(arucoDetector* detector, cv::Mat& outputImage)
{
    cv::aruco::drawDetectedMarkers(outputImage, detector->markerCorners, detector->markerIds);
    for (size_t i = 0; i < detector->markerIds.size(); i++)
    {
        Rodrigues Tvec = quaternion2rodrigues(detector->posesCov.back().at(i));
        cv::aruco::drawAxis(outputImage, detector->K, detector->D, Tvec.rvec, Tvec.tvec, detector->markerLength*1.5);
    }
}
void getDistributionAruco(arucoDetector* detector, int nPoses)
{
    if (detector->markerIds.size() != 1) {return;}

    Sophus::SE3d thisPose = quaternion2sophus(detector->posesCov.back().at(0));
    detector->distribution.posesSE3.push_back(thisPose);

    if (detector->distribution.posesSE3.size() == nPoses)
    {
        getMean(detector->distribution, 50);
        getVar(detector->distribution);
        detector->dist = false;

        std::cout << "Mean: \n" << detector->distribution.mean.matrix() << std::endl;
        std::cout << "\nVariance: \n" << detector->distribution.var << std::endl;
    }
}

void getMarkersApril(aprilDetector* detector, const precision_landing::myAprilTagDetectionArrayConstPtr& detections)
{
    if (detector->posesCov.size() == 50) {detector->posesCov.pop_front();}

    detector->posesCov.push_back(std::vector<geometry_msgs::PoseWithCovarianceStamped>());
    detector->posesCov.back().resize(detections->detections.size());


    detector->markerIds.resize(detections->detections.size());
    for (size_t i = 0; i < detections->detections.size(); i++)
    {
        detector->markerIds.at(i) = detections->detections.at(i).id[0];
    }   
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
        cv::projectPoints(objPoints, Tvec.rvec, Tvec.tvec, detector->K, detector->D, projectedPoints, J);
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
void drawMarkersApril(aprilDetector* detector, cv::Mat& outputImage)
{
    for (size_t i = 0; i < detector->posesCov.back().size(); i++)
    {
        Rodrigues Tvec = quaternion2rodrigues(detector->posesCov.back().at(i));
        cv::aruco::drawAxis(outputImage, detector->K, detector->D, Tvec.rvec, Tvec.tvec, detector->markerLength*1.5);
    }
}
void getDistributionApril(aprilDetector* detector, int nPoses)
{   
    if (detector->markerIds.size() != 1) {return;}

    Sophus::SE3d thisPose = quaternion2sophus(detector->posesCov.back().at(0));
    detector->distribution.posesSE3.push_back(thisPose);

    if (detector->posesCov.size() == nPoses)
    {
        getMean(detector->distribution, 50);
        getVar(detector->distribution);
        detector->dist = false;

        std::cout << "Mean: \n" << detector->distribution.mean.matrix() << std::endl;
        std::cout << "\nVariance: \n" << detector->distribution.var << std::endl;
    }
}

