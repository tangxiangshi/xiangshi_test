#pragma once
#include <Eigen/Dense>
class BaseEstimator
{
  public:
    // BaseEstimator(){};
    virtual void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity) {};
};
