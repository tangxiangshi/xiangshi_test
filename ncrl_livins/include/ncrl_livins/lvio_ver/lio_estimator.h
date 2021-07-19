#ifndef NCRL_LIO_ESTIMATOR_NEW_H
#define NCRL_LIO_ESTIMATOR_NEW_H
// ros msg
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

// ceres lib
#include <ceres/ceres.h>

#include "utility/utility.h"
#include "tic_toc.h"
#include "factor/marginalization_factor.h"
#include "factor/pose_local_parameterization.h"
#include "lvio_parameters.h"
#include "lvio_feature_handler.h"
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include "factor/imu_factor.h"
#include "factor/extrinsicFactor.hpp"
#include "aloam/common_modify.h"
#include "base_estimator.h"

class LioEstimator : public BaseEstimator
{
  public:
    LioEstimator();
    void processImu(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    // initial structure
    int frame_count = 0;

    Eigen::Vector3d Bas[SWEEP_SIZE + 1];
    Eigen::Vector3d Bgs[SWEEP_SIZE + 1];

    bool first_imu = false;
    IntegrationBase *tmp_pre_integration;
    Vector3d acc_0, gyr_0; // imu msg last time

};


#endif // NCRL_LIO_ESTIMATOR_NEW_H
