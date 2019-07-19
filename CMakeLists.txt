#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(realsense_ros_wrapper)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
)
catkin_package(
#  INCLUDE_DIRS include
# CATKIN_DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


set(DEPENDENCIES realsense2 ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(rs-imshow src/main.cpp)
target_link_libraries(rs-imshow ${DEPENDENCIES})

