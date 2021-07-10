#pragma once
#include<ros/ros.h>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>
#include<fstream>
#include<stdio.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PointStamped.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Imu.h>
#include<xiangshi_msg/CorrectData.h>
#include<xiangshi_msg/IMUlow.h>
#include<xiangshi_msg/PointCloud3.h>
#include<xiangshi_msg/PoseTwist.h>
#include<xiangshi_msg/Td.h>
#include"../src/aloam/common_modify.h"


using namespace std;
using namespace Eigen;

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 6; // max frame size: 6+1
const int NUM_OF_F = 1000;

// Keyframe for mapping
const double MIN_TIME = 8.0;
const float MIN_RADIUS = 1.0;
const float MIN_HEIGHT = 0.10;

// Depth association
const double DISTANCE_THRES = 0.2; // for depth association threshold
const double SQ_NORM_THRES = 25;   // for vins point search lidar point

extern std::string IMU_TOPIC;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;
extern Vector3d G;

// extern int INIT_SIZE; //for lidar
extern int ALIGN_CAMERA_TIME;
extern int ESTIMATE_LC_TD;
extern double LC_TD; //for lvio
extern double LIDAR_TD;

extern bool VINS_FRONT_USE;

// parameter for voxle filter setting
extern double lineRes;
extern double planeRes;

// extrinsic state setting
extern Vector3d ER_I2L;
extern Vector3d ET_I2L;

extern double UPWARD_ANGLE;
extern double DOWNWARD_ANGLE;
extern double CW_ANGLE;
extern double CCW_ANGLE;

// check whelther need to switch to VINS mode
constexpr double PTS_NUM_THRES = 6000;
// parameter for Maintaining map
constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};
enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

// Data type for laserodometry
typedef pair<sensor_msgs::PointCloud2ConstPtr, sensor_msgs::PointCloud2ConstPtr> FeatureType; // edge and plane(lidar feature)
typedef pair<nav_msgs::OdometryConstPtr, pair<sensor_msgs::PointCloud2ConstPtr, FeatureType>> LaserType;

// Data type for VINS
typedef pair<xiangshi_msg::PointCloud3, sensor_msgs::PointCloudConstPtr> VisionType;
typedef pair<pair<VisionType, xiangshi_msg::CorrectDataConstPtr>,
             pair<nav_msgs::OdometryConstPtr, geometry_msgs::PointStampedConstPtr>>
    VinsType;

// typedef vector<pair<vector<VinsType>, LaserType>> LvoType;
typedef pair<vector<VinsType>, LaserType> LvPairType;
typedef vector<pair<LvPairType, vector<sensor_msgs::ImuConstPtr>>> LvioType;
