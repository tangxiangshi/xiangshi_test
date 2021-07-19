#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <stdio.h>
#include "ncrl_tf.h"
#include "aloam/common_modify.h"

using namespace std;

/**
 * @brief ncrl_lio paramters
 */
const int SWEEP_SIZE = 2;
const int NUM_OF_LIDAR = 1;

extern std::string IMU_TOPIC;
extern int INIT_SIZE;//for lidar
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d G;

// check whelther need to use VINS
constexpr double PTS_NUM_THRES = 6000; 
// parameter for Maintaining map
constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

extern double LC_TD;//for lvio

// parameter for voxle filter setting
extern double lineRes;
extern double planeRes;
extern double BackgroundDis;

// extrinsic state setting
extern Eigen::Vector3d ER_I2L;
extern Eigen::Vector3d ET_I2L;

extern ncrl_tf::Trans EXTRINSIC;
//extern ncrl_tf::Trans WORLD2IMU; // WORLD -> ORIGIN -> IMU_INIT
//extern ncrl_tf::Trans IMU2GT;

void readParameters(ros::NodeHandle &n);
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
enum SolverFlag
{
    WAIT,
    INITIAL,
    NON_LINEAR
};
