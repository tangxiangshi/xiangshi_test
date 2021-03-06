#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <stdio.h>
#include <map>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "lvio_ros_msgs/CorrectData.h"
#include "lvio_ros_msgs/PointCloud3.h"
#include "lvio_ros_msgs/Td.h"

typedef std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> VinsData;
const int ENOUGH_COUNT = 5;
const double TD_ERROR_THRES = 0.002;