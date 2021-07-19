#pragma once

#include <ceres/ceres.h>
#include "ncrl_livins/point_manager.h"
#include "factor/common_factor/imu_factor.h"
#include "factor/common_factor/pose_local_parameterization.h"
#include "factor/common_factor/projection_factor_modify.h"
#include "factor/common_factor/lidar_jaco_modify.hpp"
#include "factor/depthmap_factor.h"
#include "factor/pnp_factor.h"
#include "factor/depthmap_td_factor.h"
#include "factor/pnp_td_factor.h"

class DepthMapEstimator
{
public:
    DepthMapEstimator();

    void initSpeedBias(VinsType &vins_msg);
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void setLidarTransfo(VinsType &first_vins_data, VinsType &second_vins_data, double s, int use_index);
    void exParamInit(VinsType &vins_msg);
    void processLVIO(VinsType &vins_msg, LaserType &laser_msg);
    void vector2double();
    void double2vector();
    double evaluateDepth();
    void optimization();
    void recoverDepth();
    void slideWindow();

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    SolverFlag solver_flag;

    // For IMU integration
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];

    Vector3d g;

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Td[1][1];

    int frame_count;

    double img_t[(WINDOW_SIZE + 1)];
    double laser_t[(WINDOW_SIZE + 1)];

    std_msgs::Header headers[(WINDOW_SIZE + 1)];

    bool first_imu;
    bool add_imu;

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    PointManager pts_manager;

    // Output for debugging
    PtMap predict_f_pts_cj;             // predict the features position in camera frame at lj moment
    PtMap solved_pts;                   // correct the image feature
    vector<vector<Vector3d>> f_arrow_l; // check the failure points of depth association
    int received_num;
};
