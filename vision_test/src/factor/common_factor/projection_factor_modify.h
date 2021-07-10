#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/point_utility.h"

#if 1
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 1>
{
public:
  ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, const PointUtility::Tf &_transfo_b_c);
  // ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, const PointUtility::Tf &_transfo_b_c, const double &_inv_dep);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;

  PointUtility::Tf transfo_b_c;

  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};
#else
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7>
{
public:
  // ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, const PointUtility::Tf &_transfo_b_c);
  ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, const PointUtility::Tf &_transfo_b_c, const double &_inv_dep);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;

  PointUtility::Tf transfo_b_c;

  double inv_dep_j;

  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};
#endif