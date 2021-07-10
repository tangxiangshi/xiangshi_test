#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/tic_toc.h"
#include "../utility/utility.h"
#include "../utility/point_utility.h"

class DepthMapFactor : public ceres::SizedCostFunction<3, 7, 7, 1>
{
public:
    DepthMapFactor(const Eigen::Vector3d _pts_l, const Eigen::Vector3d _pts_m, const Eigen::Vector3d _pts_n,
                   const Eigen::Vector3d _pts_j, const PointUtility::Tf &_transfo_b_l, const PointUtility::Tf &_transfo_b_c);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);
    double checkResidual(double **parameters);

    // measurements
    Eigen::Vector3d pts_l;
    Eigen::Vector3d pts_m;
    Eigen::Vector3d pts_n; // Lidar flat points
    Eigen::Vector3d nu;

    Eigen::Vector3d pts_j; // visual point

    // tmp params
    PointUtility::Tf transfo_b_l;
    PointUtility::Tf transfo_b_c;

    static Eigen::Matrix3d sqrt_info;
    static double sum_t;
};
