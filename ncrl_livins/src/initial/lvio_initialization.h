#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include "factor/integration_base.h"
#include "utility/utility.h"
#include "ncrl_livins/ncrl_tf.h"

using namespace Eigen;
using namespace std;

class LaserFrame
{
  public:
    LaserFrame(){};
    LaserFrame(const ncrl_tf::Trans &_Tinit2odom_I, double _t) : t{_t}
    {
        Tinit2odom_I = _Tinit2odom_I;
    }
    ncrl_tf::Trans Tinit2odom_I;// translate lidar poses to IMU frame
    double t;
    // int id;
    // LvioIntegrationBase *lvio_pre_integration;
    IntegrationBase *pre_integration;  
};

bool LaserImuAlignment(map<int, LaserFrame> &all_laser_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);
