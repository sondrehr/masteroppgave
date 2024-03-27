#include <ros/ros.h>
#include "classes.h"

//#include "boost/filesystem.hpp"
//std::cout << boost::filesystem::current_path() << std::endl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;

    arucoDetector arucoDet(nh);

    ros::spin();
    return 0;
}
