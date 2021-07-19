#pragma once

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include "ncrl_livins/depth_map_estimator.h"
#include "lvio_ros_msgs/CorrectData.h"

using namespace std;

typedef pcl::PointXYZI PointType;

void registerPub(ros::NodeHandle &n);

void pubCorrectCloud(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubVinsCloudFiltered(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubGuessCloud(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubVisualization(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubLidarCurrent(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubSurroundingCloud(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubLIVOdometry(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubGroundTruth(const geometry_msgs::PoseStampedConstPtr &gt_msg);

void pubVINSPose(const lvio_ros_msgs::CorrectDataConstPtr &state);

void pubTf(DepthMapEstimator &estimator, const std_msgs::Header &header);

void pubReceivedVisionNum(DepthMapEstimator &estimator, const std_msgs::Header &header);