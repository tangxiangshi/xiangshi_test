#pragma once
#include <ros/ros.h>
#include <map>
#include "lvio_parameters.h"
#include "lvio_feature_handler.h"
#include "initial/lvio_initialization.h"
#include "factor/imu_factor.h"
// #include "base_estimator.h"

class LvioEstimator
{
  public:
    LvioEstimator();

    void setParameter();
    void clearState();

    // imu use
    void processImu(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processLVIO(FeaturePerSweep &laser_msg, int lidar_id);
    bool initialStructure();
    bool laserInitialAlign();

    enum SolverFlag
    {
        WAIT,
        INITIAL,
        NON_LINEAR
    };
    SolverFlag solver_flag;
    
    int frame_count;//for lidar
    double lidar_td;
    double t_last = 0;//for initialization
    bool init_lvio = true;//init first init stamp
    // std_msgs::Header Headers[(INIT_SIZE + 1)];

    // vision pointcloud
    // map<int, vector<Eigen::Matrix<double, 7, 1>>> image;//id->feature data 

    map<int, LaserFrame> all_laser_frame;//for initialization

    // eigen state
    Eigen::Vector3d Ps[SWEEP_SIZE + 1];
    Eigen::Quaterniond Rs[SWEEP_SIZE + 1];
    Eigen::Vector3d Vs[SWEEP_SIZE + 1];
    Eigen::Vector3d Bas[SWEEP_SIZE + 1];
    Eigen::Vector3d Bgs[SWEEP_SIZE + 1];
    Eigen::Quaterniond qil[1];
    Eigen::Vector3d til[1];
    Eigen::Quaterniond w2init[1];

    // process imu varibale
    bool first_imu = false;
    IntegrationBase *tmp_pre_integration;
    Vector3d acc_0, gyr_0; // imu msg last time
    ncrl_tf::Point g;

    laser_feature_handler *laser_feature_all;
};
