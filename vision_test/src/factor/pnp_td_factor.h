#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/point_utility.h"
#include "../utility/tic_toc.h"

class PnpTdFactor : public ceres::SizedCostFunction<2, 1, 1>
{
public:
    PnpTdFactor(const PointUtility::Tf &_transfo_li_lj, const PointUtility::Tf &_transfo_l_c, const Eigen::Vector3d &_pts_j,
                const Eigen::Vector3d &_f_pts_li, const Eigen::Vector2d &_velocity_j, const double &_cur_td);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);
    double checkResidual(double **parameters);

    // tmp params
    // loam frame to frame odometry
    PointUtility::Tf transfo_li_lj;
    // extrinsic parameter between lidar and camera
    PointUtility::Tf transfo_l_c;

    // measurements
    Eigen::Vector3d pts_j; // visual point
    Eigen::Vector3d f_pts_li;
    Eigen::Vector3d velocity_j;
    double td_j;

    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};