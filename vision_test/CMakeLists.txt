cmake_minimum_required(VERSION 3.0.2)
project(vision_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  rospy
  tf
  image_transport
  visualization_msgs
  cv_bridge
  genmsg
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_message_files(DIRECTORY ../lvio_ros_msgs/msg FILES
CorrectData.msg
IMUlow.msg
PointCloud3.msg
PoseTwist.msg
Td.msg
)

generate_messages(DEPENDENCIES
std_msgs
geometry_msgs
sensor_msgs)

catkin_package(
DEPENDS EIGEN3 PCL
)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}  
)


add_executable(vio 
src/vio.cpp)

target_link_libraries(vio ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
# add_dependencies(vision_test vision_test_generate_messages_cpp)





