#include <map>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "vision_test/depth_map_parameters.h"
#include "vision_test/depth_map_estimator.h"
#include "utility/depth_map_visualization.h"
#include "utility/ncrl_tf.h"
#include "xiangshi_msgs/CorrectData.h"
#include "xiangshi_msgs/Td.h"

using namespace std;
using namespace message_filters;