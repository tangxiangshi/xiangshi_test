#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "lvio_estimator.h"
// #include "lio_estimator.h"
#include "lvio_ros_msgs/CorrectData.h"
// #include "lvio_ros_msgs/IMUlow.h"
#include "lvio_ros_msgs/PoseTwist.h"
#include "lvio_ros_msgs/PointCloud3.h"
#include "lvio_ros_msgs/Td.h"

using namespace std;
using namespace message_filters;

/*====Switch====*/
// #define DEBUG

// Data type for laserodometry
typedef pcl::PointXYZI PointType;
typedef std::pair<sensor_msgs::PointCloud2ConstPtr, sensor_msgs::PointCloud2ConstPtr> FeatureType;// edge and plane(lidar feature)
typedef std::pair<nav_msgs::OdometryConstPtr,std::pair<sensor_msgs::PointCloud2ConstPtr, FeatureType>> LaserType;

// Data type for VINS
typedef std::pair<lvio_ros_msgs::PointCloud3, sensor_msgs::PointCloudConstPtr> VisionType;
typedef std::pair<std::pair<VisionType, lvio_ros_msgs::PoseTwist>, nav_msgs::OdometryConstPtr> VinsType;
typedef std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<VinsType>> CorrectDataType;

// lvio
typedef std::vector<std::pair<CorrectDataType,LaserType>> LvioData;

const double VEL_TRES = 0.03;
