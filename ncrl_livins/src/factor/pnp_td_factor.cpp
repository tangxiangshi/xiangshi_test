#include "pnp_td_factor.h"
// #define UNIT_SPHERE_ERROR

Eigen::Matrix2d PnpTdFactor::sqrt_info = 460.0 / 1.5 * Eigen::Matrix2d::Identity();
double PnpTdFactor::sum_t;

PnpTdFactor::PnpTdFactor(const PointUtility::Tf &_transfo_li_lj, const PointUtility::Tf &_transfo_l_c,
                         const Eigen::Vector3d &_pts_j, const Eigen::Vector3d &_f_pts_li,
                         const Eigen::Vector2d &_velocity_j, const double &_cur_td)
    : transfo_li_lj(_transfo_li_lj), transfo_l_c(_transfo_l_c), pts_j(_pts_j), f_pts_li(_f_pts_li), td_j(_cur_td)
{
    velocity_j.x() = _velocity_j.x();
    velocity_j.y() = _velocity_j.y();
    velocity_j.z() = 0;
};

bool PnpTdFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
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
    
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    // 1th
    Eigen::Vector3d cloud_pts_ci = transfo_l_c.q.inverse() * (f_pts_li - transfo_l_c.t);
    Eigen::Vector3d img_pts_ci = transfo_l_c.q.inverse() * (img_pts_li - transfo_l_c.t);

#ifdef UNIT_SPHERE_ERROR
    Eigen::Matrix<double, 2, 3> tangent_base;
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = cloud_pts_ci.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();

    residual = tangent_base * (cloud_pts_ci.normalized() - img_pts_ci.normalized());
#else
    residual = (cloud_pts_ci / cloud_pts_ci.z()).head(2) - (img_pts_ci / img_pts_ci.z()).head(2);
#endif

    residual = sqrt_info * residual;
    // std::cout << "init distance:" << residuals[0] << std::endl;

    if (jacobians)
    {
        // Jacobian scale
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
#ifdef UNIT_SPHERE_ERROR
        double norm = img_pts_ci.norm();
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = img_pts_ci(0);
        x2 = img_pts_ci(1);
        x3 = img_pts_ci(2);
        norm_jaco << -1.0 / norm + x1 * x1 / pow(norm, 3), x1 * x2 / pow(norm, 3), x1 * x3 / pow(norm, 3),
            x1 * x2 / pow(norm, 3), -1.0 / norm + x2 * x2 / pow(norm, 3), x2 * x3 / pow(norm, 3),
            x1 * x3 / pow(norm, 3), x2 * x3 / pow(norm, 3), -1.0 / norm + x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else
        reduce << -1. / img_pts_ci(2), 0, img_pts_ci(0) / (img_pts_ci(2) * img_pts_ci(2)),
            0, -1. / img_pts_ci(2), img_pts_ci(1) / (img_pts_ci(2) * img_pts_ci(2));
#endif

        reduce = sqrt_info * reduce;

        /*****Jacobian derivation*****/
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[0]);
            Eigen::Vector3d tmp_jaco = transfo_l_c.q.inverse() * transfo_li_lj.q * transfo_l_c.q * (pts_j - (td - td_j) * velocity_j) * -1.0 / (inv_dep_j * inv_dep_j);
            jacobian_feature = reduce * tmp_jaco;

            // Eigen::JacobiSVD<Eigen::Vector2d> svd(jacobian_feature, 0);
            // std::cout << "singular values of depth are:" << std::endl << svd.singularValues() << std::endl;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[1]);
            Eigen::Vector3d tmp_jaco = transfo_l_c.q.inverse() * transfo_li_lj.q * transfo_l_c.q * velocity_j / inv_dep_j * -1.0;
            jacobian_td = reduce * tmp_jaco;

            // Eigen::JacobiSVD<Eigen::Vector2d> svd(jacobian_td);
            // std::cout << "singular values of td are:" << std::endl << svd.singularValues() << std::endl;
        }
    }
    sum_t += tic_toc.toc();

    return true;
}

void PnpTdFactor::check(double **parameters)
{
    double *res = new double[2];
    double **jaco = new double *[2];
    jaco[0] = new double[2 * 1];
    jaco[1] = new double[2 * 1];
    Evaluate(parameters, res, jaco);
    puts("check begins");

    puts("my Jacobian for td");

    // std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(res).transpose() << std::endl
    //           << std::endl;
    // std::cout << Eigen::Map<Eigen::Vector3d>(jaco[0]) << std::endl
    //           << std::endl;
    std::cout << Eigen::Map<Eigen::Vector2d>(jaco[1]) << std::endl
              << std::endl;
}