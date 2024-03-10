#include "functions.h"


void detectMarkersAruco(arucoDetector* detector, cv::Mat& inputImage)
{
    cv::aruco::detectMarkers(inputImage, detector->dictionary, detector->markerCorners, detector->markerIds);
    detector->Tvecs.resize(detector->markerIds.size());
}


void drawMarkersAruco(arucoDetector* detector, cv::Mat& outputImage)
{
    cv::aruco::drawDetectedMarkers(outputImage, detector->markerCorners, detector->markerIds);
    for (size_t i = 0; i < detector->markerIds.size(); i++)
    {
        cv::aruco::drawAxis(outputImage, detector->K, detector->D, detector->Tvecs.at(i).rvec, detector->Tvecs.at(i).tvec, detector->markerLength*1.5);
    }
}


void estimatePoseAruco(arucoDetector* detector)
{
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-detector->markerLength/2, detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(detector->markerLength/2, detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(detector->markerLength/2, -detector->markerLength/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-detector->markerLength/2, -detector->markerLength/2, 0);

    for (size_t i = 0; i < detector->markerIds.size(); i++)
    {
        cv::solvePnP(objPoints, detector->markerCorners.at(i), detector->K, detector->D, detector->Tvecs.at(i).rvec, detector->Tvecs.at(i).tvec);
        
        // Test!!!
        //cv::solvePnPRefineLM(objPoints, detector->markerCorners.at(i), detector->K, detector->D, detector->Tvecs.at(i).rvec, detector->Tvecs.at(i).tvec);
        //cv::solvePnPRefineVVS(objPoints, detector->markerCorners.at(i), detector->K, detector->D, detector->Tvecs.at(i).rvec, detector->Tvecs.at(i).tvec);
    }
}


