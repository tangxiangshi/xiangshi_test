#include "projection_factor_modify.h"

Eigen::Matrix2d ProjectionFactor::sqrt_info = 460.0 / 1.5 * Eigen::Matrix2d::Identity();
double ProjectionFactor::sum_t;

#if 1
ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i,
                                   const Eigen::Vector3d &_pts_j,
                                   const PointUtility::Tf &_transfo_b_c) : pts_i(_pts_i),
                                                                           pts_j(_pts_j),
                                                                           transfo_b_c(_transfo_b_c)
{
    // initialization
};

bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    double inv_dep_j = parameters[2][0];

    Eigen::Vector3d pts_camera_j = pts_j / inv_dep_j;
    Eigen::Vector3d pts_imu_j = transfo_b_c.q * pts_camera_j + transfo_b_c.t;
    Eigen::Vector3d pts_w = Qj * pts_imu_j + Pj;
    Eigen::Vector3d pts_imu_i = Qi.inverse() * (pts_w - Pi);
    Eigen::Vector3d pts_camera_i = transfo_b_c.q.inverse() * (pts_imu_i - transfo_b_c.t);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    double dep_i = pts_camera_i.z();
    residual = pts_i.head<2>() - (pts_camera_i / dep_i).head<2>();

    residual = sqrt_info * residual;

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = transfo_b_c.q.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << -1. / dep_i, 0, pts_camera_i(0) / (dep_i * dep_i),
            0, -1. / dep_i, pts_camera_i(1) / (dep_i * dep_i);

        reduce = sqrt_info * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Ri.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = -ric.transpose() * Ri.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Ri.transpose() * Rj * Utility::skewSymmetric(pts_camera_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }

        if (jacobians[2])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
            jacobian_feature = reduce * ric.transpose() * Ri.transpose() * Rj * ric * pts_j * 1.0 / (inv_dep_j * inv_dep_j);
        }
    }
    sum_t += tic_toc.toc();

    return true;
}
#else
ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i,
                                   const Eigen::Vector3d &_pts_j,
                                   const PointUtility::Tf &_transfo_b_c,
                                   const double &_inv_dep) : pts_i(_pts_i),
                                                             pts_j(_pts_j),
                                                             transfo_b_c(_transfo_b_c),
                                                             inv_dep_j(_inv_dep)
{
    // initialization
};

bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    // double inv_dep_j = parameters[2][0];

    Eigen::Vector3d pts_camera_j = pts_j / inv_dep_j;
    Eigen::Vector3d pts_imu_j = transfo_b_c.q * pts_camera_j + transfo_b_c.t;
    Eigen::Vector3d pts_w = Qj * pts_imu_j + Pj;
    Eigen::Vector3d pts_imu_i = Qi.inverse() * (pts_w - Pi);
    Eigen::Vector3d pts_camera_i = transfo_b_c.q.inverse() * (pts_imu_i - transfo_b_c.t);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    double dep_i = pts_camera_i.z();
    residual = pts_i.head<2>() - (pts_camera_i / dep_i).head<2>();

    residual = sqrt_info * residual;

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = transfo_b_c.q.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << -1. / dep_i, 0, pts_camera_i(0) / (dep_i * dep_i),
            0, -1. / dep_i, pts_camera_i(1) / (dep_i * dep_i);

        reduce = sqrt_info * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Ri.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = -ric.transpose() * Ri.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Ri.transpose() * Rj * Utility::skewSymmetric(pts_camera_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }

        // if (jacobians[2])
        // {
        //     Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
        //     jacobian_feature = reduce * ric.transpose() * Ri.transpose() * Rj * ric * pts_j * 1.0 / (inv_dep_j * inv_dep_j);
        // }
    }
    sum_t += tic_toc.toc();

    return true;
}
#endif