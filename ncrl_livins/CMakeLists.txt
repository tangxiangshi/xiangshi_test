cmake_minimum_required(VERSION 3.0.2)
project(ncrl_livins)

## Normal Mode
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Debug Mode
#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

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
#message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_message_files(DIRECTORY ../lvio_ros_msgs/msg FILES PointCloud3.msg 
                                                       CorrectData.msg IMUlow.msg PoseTwist.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
DEPENDS EIGEN3 PCL
)

include_directories(
  include
  src
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## SCANREGISTRATION AND LASERODOMETRY
add_executable(scanRegistration src/aloam/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(laserOdometry src/aloam/laserOdometry.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(laserMapping src/aloam/laserMappingTest.cpp
              src/factor/common_factor/pose_local_parameterization.cpp
              )
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(depth_map 
    src/depth_map_node.cpp
    src/depth_map_parameters.cpp
    src/depth_map_estimator.cpp
    src/point_manager.cpp
    src/factor/common_factor/pose_local_parameterization.cpp
    src/factor/common_factor/projection_factor_modify.cpp
    src/factor/depthmap_factor.cpp
    src/factor/pnp_factor.cpp    
    src/utility/depth_map_visualization.cpp
    #src/factor/depthmap_td_factor.cpp
    #src/factor/pnp_td_factor.cpp
)
target_link_libraries(depth_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(depth_map ncrl_livins_generate_messages_cpp)

add_executable(ground_truth
    src/gt_helper_node.cpp
)
target_link_libraries(ground_truth ${catkin_LIBRARIES})

add_executable(sync
    src/sync.cpp
)
target_link_libraries(sync ${catkin_LIBRARIES})
