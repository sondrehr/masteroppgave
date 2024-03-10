cmake_minimum_required(VERSION 3.0.2)
project(subscriber)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

# Find vars
find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  image_transport
  OpenCV
  cv_bridge
)


# To use rosrun
catkin_package()

# Include .h files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Get .exe file
add_executable(aruco src/aruco.cpp src/functions.cpp src/lie.cpp src/classes.cpp)
target_link_libraries(aruco 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(april src/april.cpp src/functions.cpp src/lie.cpp src/classes.cpp)
target_link_libraries(april
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

