#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/point_utility.h"
#include "../utility/tic_toc.h"

class PnpFactor : public ceres::SizedCostFunction<2, 7>
{
public:
    PnpFactor(const PointUtility::Tf &_transfo_b_c, const Eigen::Vector3d &_pts_j,
              const Eigen::Vector3d &_f_pts_w, const double &_inv_dep);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    bool checkSingularVal(double **parameters);
    
    // extrinsic parameter between lidar and camera
    PointUtility::Tf transfo_b_c;

    // measurements
    Eigen::Vector3d pts_j; // visual point
    Eigen::Vector3d f_pts_w;
    double inv_dep_j;

    // static Eigen::Matrix3d sqrt_info;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};