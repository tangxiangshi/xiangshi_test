#include "vision_test/lio_estimator.h"

LioEstimator::LioEstimator()
{
    ROS_INFO("LVIO init begins");
}

void LioEstimator::processImu(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu){
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    // initial
    if(!tmp_pre_integration)
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[SWEEP_SIZE], Bgs[SWEEP_SIZE]};

    // ignore the first combined data, since the datas aren't sufficient in 0.1 sec
    if (frame_count != 0) // frame_count 1 2 3 ... 9 10
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
