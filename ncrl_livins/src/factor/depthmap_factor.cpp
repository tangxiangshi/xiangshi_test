#include "depthmap_factor.h"

// Eigen::Matrix3d DepthMapFactor::sqrt_info = Eigen::Matrix3d::Identity();
Eigen::Matrix3d DepthMapFactor::sqrt_info = 460.0 / 1.5 * Eigen::Matrix3d::Identity();

double DepthMapFactor::sum_t;

DepthMapFactor::DepthMapFactor(const Eigen::Vector3d _pts_l, const Eigen::Vector3d _pts_m, const Eigen::Vector3d _pts_n,
                               const Eigen::Vector3d _pts_j, const PointUtility::Tf &_transfo_b_l, const PointUtility::Tf &_transfo_b_c)
    : pts_l(_pts_l), pts_m(_pts_m), pts_n(_pts_n), pts_j(_pts_j), transfo_b_l(_transfo_b_l), transfo_b_c(_transfo_b_c)
{
    nu = (pts_n - pts_m).cross(pts_n - pts_l);
    nu.normalized();
};

bool DepthMapFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;

    // 1th parameter
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    // 2th parameter
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    // 3th parameter
    double inv_dep_j = parameters[2][0];

    // Image points are reprojected to li frame
    Eigen::Vector3d img_pts_cj = pts_j / inv_dep_j;
    Eigen::Vector3d img_pts_bj = transfo_b_c.q * img_pts_cj + transfo_b_c.t;
    Eigen::Vector3d img_pts_w = Qj * img_pts_bj + Pj;
    Eigen::Vector3d img_pts_bi = Qi.inverse() * (img_pts_w - Pi);
    Eigen::Vector3d img_pts_li = transfo_b_l.q.inverse() * (img_pts_bi - transfo_b_l.t);

    Eigen::Map<Eigen::Vector3d> residual(residuals);

    sqrt_info << 1., 0, 0,
        0, 460.0 / 1.5, 0,
        0, 0, 460.0 / 1.5;

    // 1th
    residual = sqrt_info * nu.dot(pts_n - img_pts_li) * nu;
    // std::cout << "init distance:" << residuals[0] << std::endl;

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d R_b_l = transfo_b_l.q.toRotationMatrix();

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;

            // n*n^T * (R{_l^b}^T * R{_bi^w}^T)
            jaco_i.leftCols<3>() = R_b_l.transpose() * Ri.transpose();
            // n*n^T * (R{_l^b}^T * R{_bi^w}^T) * -[R{_bj^w} * (R{_c^b} * (pts_j / inv_dep_j) + t_{_c^b}) + t_{_w^bj} - t_{_w^bi}]_x
            jaco_i.rightCols<3>() = R_b_l.transpose() * -Utility::skewSymmetric(img_pts_bi);

            jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;

            // -n*n^T * (R{_l^b}^T * R{_bi^w}^T)
            jaco_j.leftCols<3>() = -R_b_l.transpose() * Ri.transpose(); // position-term
            // n*n^T * (R{_l^b}^T * R{_bi^w}^T) * R{_bj^w} * [R{_c^b} * (pts_j / inv_dep_j) + t_{_c^b}]_x
            jaco_j.rightCols<3>() = R_b_l.transpose() * Ri.transpose() * Rj * Utility::skewSymmetric(img_pts_bj); // orientation-term

            jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero(); // quaternion-w
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Vector3d> jacobian_feature(jacobians[2]);
            jacobian_feature = sqrt_info * nu.dot(transfo_b_l.q.inverse() * Ri.transpose() * Rj * transfo_b_c.q * pts_j * 1.0 / (inv_dep_j * inv_dep_j)) * nu;
            // Eigen::JacobiSVD<Eigen::Vector3d> svd(jacobian_feature, 0);
            // std::cout << "singular values of depth are:" << std::endl << svd.singularValues() << std::endl;
        }
    }
    sum_t += tic_toc.toc();

    return true;
}

double DepthMapFactor::checkResidual(double **parameters)
{
    // 1th parameter
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    // 2th parameter
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    // 3th parameter
    double inv_dep_j = parameters[2][0];

    // Image points are reprojected to li frame
    Eigen::Vector3d img_pts_cj = pts_j / inv_dep_j;
    Eigen::Vector3d img_pts_bj = transfo_b_c.q * img_pts_cj + transfo_b_c.t;
    Eigen::Vector3d img_pts_w = Qj * img_pts_bj + Pj;
    Eigen::Vector3d img_pts_bi = Qi.inverse() * (img_pts_w - Pi);
    Eigen::Vector3d img_pts_li = transfo_b_l.q.inverse() * (img_pts_bi - transfo_b_l.t);

    // 1th
    return nu.dot(img_pts_li - pts_n);
}