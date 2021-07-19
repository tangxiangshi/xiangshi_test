#include "ncrl_livins/lvio_estimator.h"

LvioEstimator::LvioEstimator()
{
    ROS_INFO("LVIO init begins");
    laser_feature_all = new laser_feature_handler;
    clearState();
}

void LvioEstimator::clearState()
{
    for (int i = 0; i < SWEEP_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
    }
    solver_flag = INITIAL;
    frame_count = 0;
    lidar_td = LC_TD;
    til[0] = Eigen::Vector3d::Zero();
    qil[0] = Eigen::Quaterniond::Identity();
    w2init[0] = Eigen::Quaterniond::Identity();
    acc_0 = Eigen::Vector3d::Zero();
    gyr_0 = Eigen::Vector3d::Zero();
    ncrl_tf::setPointFrame("WORLD", g);
    ncrl_tf::setPoint(G, g);
    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    tmp_pre_integration = nullptr;
}

void LvioEstimator::processImu(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    // initial
    if (!tmp_pre_integration)
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[SWEEP_SIZE], Bgs[SWEEP_SIZE]};

    // ignore the first combined data, since the datas aren't sufficient in 0.1 sec
    if (frame_count != 0) // frame_count 1 2 3 ... 9 10
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void LvioEstimator::processLVIO(FeaturePerSweep &laser_msg, int lidar_id)
{
    // Headers[frame_count] = laser_msg.laserOdom.header.stamp.toSec();
    if (init_lvio)
    {
        t_last = laser_msg.laserOdom.header.stamp.toSec();
        init_lvio = false;
    }
    else
    {
        // if missing data, need to re-initialize
        if (laser_msg.laserOdom.header.stamp.toSec() - t_last > 0.11)
        {
            all_laser_frame.erase(all_laser_frame.begin(), all_laser_frame.end());
            frame_count = 0;
        }
        t_last = laser_msg.laserOdom.header.stamp.toSec();
    }

    LaserFrame laser_frame(laser_msg.Tinit2odom_I, laser_msg.laserOdom.header.stamp.toSec());
    laser_frame.pre_integration = tmp_pre_integration;
    all_laser_frame.insert(make_pair(lidar_id, laser_frame));

    if (solver_flag == INITIAL)
    {
        if (initialStructure())
        {
            frame_count = SWEEP_SIZE;
            // updateState()
            solver_flag = NON_LINEAR;
        }
        else
        {
            ROS_INFO_STREAM("ACCUMULATE THE COMBINED DATE WITH " << frame_count);
            frame_count++;
        }
    }
    else
    {
        // updateState()
    }

    if (solver_flag == NON_LINEAR)
    {
        // optimization()
        // maintainSize()
    }
}

bool LvioEstimator::initialStructure()
{
    if (frame_count >= INIT_SIZE)
    {
        int size = all_laser_frame.size();
        if (size != INIT_SIZE + 1)
            ROS_WARN_STREAM("SIZE IN INITIALSTRUCTURE IS " << size << "! IT SHOULD BE " << INIT_SIZE + 1);

        return laserInitialAlign();
    }
    else
        return false;
}
bool LvioEstimator::laserInitialAlign()
{
    Eigen::VectorXd x;
    bool result = LaserImuAlignment(all_laser_frame, Bgs, g.point, x);
    if (!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // calibrate the rotation from world to imu and set the rotation as initial state
    // g.frame = "IMU_INIT";
    // Eigen::Quaterniond q_ = Eigen::Quaterniond::FromTwoVectors(g.point, G);
    // Eigen::Vector3d euler = Q2RPY(q_);

    // // update state from imu init frame to world frame
    // WORLD2IMU.v = q_ * WORLD2IMU.v;
    // WORLD2IMU.q = q_ * WORLD2IMU.q;
    // ROS_DEBUG_STREAM("EULER CALIBRATION FROM WORLD TO IMU_INIT : " << rad2deg(euler).transpose() << " deg");
    // Feature_All->initSizeMaintainer();

    return result;
}