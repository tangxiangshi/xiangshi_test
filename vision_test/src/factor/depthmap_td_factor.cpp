#include "depthmap_td_factor.h"

Eigen::Matrix3d DepthMapTdFactor::sqrt_info = Eigen::Matrix3d::Identity();
// Eigen::Matrix3d DepthMapTdFactor::sqrt_info = 460.0 / 1.5 * Eigen::Matrix3d::Identity();

double DepthMapTdFactor::sum_t;

DepthMapTdFactor::DepthMapTdFactor(const PointUtility::Tf &_transfo_li_lj,
                                   const PointUtility::Tf &_transfo_l_c, const Eigen::Vector3d &_pts_j,
                                   const Eigen::Vector3d &_nu, const Eigen::Vector3d &_pts_n,
                                   const Eigen::Vector2d &_velocity_j, const double &_cur_td)
    : transfo_li_lj(_transfo_li_lj), transfo_l_c(_transfo_l_c), pts_j(_pts_j), nu(_nu), pts_n(_pts_n), td_j(_cur_td)
{
    velocity_j.x() = _velocity_j.x();
    velocity_j.y() = _velocity_j.y();
    velocity_j.z() = 0;
};

bool DepthMapTdFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;

    // 1th parameter
    const double inv_dep_j = parameters[0][0];

    // 2th parameter
    double td = parameters[1][0];

    // Image points are reprojected to li frame
    Eigen::Vector3d pts_j_td = pts_j - (td - td_j) * velocity_j;
    Eigen::Vector3d img_pts_cj = pts_j_td / inv_dep_j;
    Eigen::Vector3d img_pts_lj = transfo_l_c.q * img_pts_cj + transfo_l_c.t;
    Eigen::Vector3d img_pts_li = transfo_li_lj.q * img_pts_lj + transfo_li_lj.t;
    
    Eigen::Map<Eigen::Vector3d> residual(residuals);

    sqrt_info << 1., 0, 0,
        0, 460.0 / 1.5, 0,
        0, 0, 460.0 / 1.5;

    // 1th
    residual = sqrt_info * nu.dot(img_pts_li - pts_n) * nu;
    // std::cout << "init distance:" << residuals[0] << std::endl;

    if (jacobians)
    {
        // Jacobian scale
        /*****Jacobian derivation*****/
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Vector3d> jacobian_feature(jacobians[0]);
            jacobian_feature = sqrt_info * nu.dot(transfo_li_lj.q * transfo_l_c.q * (pts_j - (td - td_j) * velocity_j) * -1.0 / (inv_dep_j * inv_dep_j)) * nu;
            // Eigen::JacobiSVD<Eigen::Vector3d> svd(jacobian_feature, 0);
            // std::cout << "singular values of depth are:" << std::endl << svd.singularValues() << std::endl;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Vector3d> jacobian_td(jacobians[1]);
            jacobian_td = sqrt_info * nu.dot(transfo_li_lj.q * transfo_l_c.q * velocity_j / inv_dep_j * -1.0) * nu;
            // Eigen::JacobiSVD<Eigen::Vector3d> svd(jacobian_td, 0);
            // std::cout << "singular values of td are:" << std::endl << svd.singularValues() << std::endl;
        }
    }
    sum_t += tic_toc.toc();

    return true;
}

void DepthMapTdFactor::check(double **parameters)
{
    double *res = new double[3];
    double **jaco = new double *[2];
    jaco[0] = new double[3 * 1];
    jaco[1] = new double[3 * 1];
    Evaluate(parameters, res, jaco);
    // puts("check begins");

    // puts("my Jacobian for td");

    // std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(res).transpose() << std::endl
    //           << std::endl;
    // std::cout << Eigen::Map<Eigen::Vector3d>(jaco[0]) << std::endl
    //           << std::endl;
    // std::cout << Eigen::Map<Eigen::Vector3d>(jaco[1]) << std::endl
    //           << std::endl;

    // // 1th parameter
    // double inv_dep_j = parameters[0][0];

    // // 2th parameter
    // double td = parameters[1][0];

    // Eigen::Vector3d nu = (pts_m - pts_n).cross(pts_l - pts_n);
    // nu.normalize();
    // Eigen::Vector3d pts_j_td = pts_j - (td - td_j) * velocity_j;
    // Eigen::Vector3d pts_camera_j = pts_j_td / inv_dep_j;
    // Eigen::Vector3d pts_lj2cj = qlc * pts_camera_j + tlc;
    // Eigen::Vector3d pts_li2cj = q_li2lj * pts_lj2cj + t_li2lj;
    // Eigen::Vector3d residual;

    // // 1th
    // residual = sqrt_info * nu.dot(pts_li2cj - pts_n) * nu;

    // puts("num");
    // std::cout << residual << std::endl;

    // const double eps = 1e-6;
    // Eigen::Matrix<double, 3, 20> num_jacobian;
    // for (int k = 0; k < 20; k++)
    // {
    //     // 1th parameter
    //     double inv_dep_j = parameters[0][0];

    //     // 2th parameter
    //     double td = parameters[1][0];

    //     int b = k % 3;
    //     Eigen::Vector2d delta = Eigen::Vector2d(b == 0, b == 1) * eps;

    //     if (b == 0)
    //         inv_dep_j += delta.x();
    //     else if (b == 1)
    //         td += delta.y();

    //     Eigen::Vector3d nu = (pts_m - pts_n).cross(pts_l - pts_n);
    //     Eigen::Vector3d pts_j_td = pts_j - (td - td_j) * velocity_j;
    //     Eigen::Vector3d pts_camera_j = pts_j_td / inv_dep_j;
    //     Eigen::Vector3d pts_lj2cj = qlc * pts_camera_j + tlc;
    //     Eigen::Vector3d pts_li2cj = q_li2lj * pts_lj2cj + t_li2lj;
    //     Eigen::Vector3d tmp_residual;

    //     // 1th
    //     tmp_residual = sqrt_info * nu.dot(pts_li2cj - pts_n) * nu;

    //     num_jacobian.col(k) << (tmp_residual - residual) / eps;
    // }
    // std::cout << num_jacobian << std::endl;
}

double DepthMapTdFactor::checkResidual(double **parameters)
{
    // 1th parameter
    double inv_dep_j = parameters[0][0];

    // 2th parameter
    double td = parameters[1][0];

    Eigen::Vector3d pts_j_td = pts_j - (td - td_j) * velocity_j;
    Eigen::Vector3d img_pts_cj = pts_j_td / inv_dep_j;
    Eigen::Vector3d img_pts_lj = transfo_l_c.q * img_pts_cj + transfo_l_c.t;
    Eigen::Vector3d img_pts_li = transfo_li_lj.q * img_pts_lj + transfo_li_lj.t;

    // 1th
    return nu.dot(img_pts_li - pts_n);
}