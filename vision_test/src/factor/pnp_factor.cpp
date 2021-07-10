#include "pnp_factor.h"
// #define UNIT_SPHERE_ERROR

Matrix2d PnpFactor::sqrt_info = 460.0 / 1.5 * Eigen::Matrix2d::Identity();
// Matrix3d PnpFactor::sqrt_info = Matrix3d::Identity();
double PnpFactor::sum_t;

PnpFactor::PnpFactor(const PointUtility::Tf &_transfo_b_c, const Eigen::Vector3d &_pts_j,
                     const Eigen::Vector3d &_f_pts_w, const double &_inv_dep)
    : transfo_b_c(_transfo_b_c), pts_j(_pts_j), f_pts_w(_f_pts_w), inv_dep_j(_inv_dep){};

bool PnpFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;

    // 1th parameter
    Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);
    Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    // Image points are reprojected to li frame
    // Vector3d img_pts_cj = pts_j / inv_dep_j;
    // Vector3d img_pts_bj = transfo_b_c.q * img_pts_cj + transfo_b_c.t;
    // Vector3d img_pts_w = Qj * img_pts_bj + Pj;
    Vector3d img_pts_bj = Qj.inverse() * (f_pts_w - Pj);
    Vector3d img_pts_camera = transfo_b_c.q.inverse() * (img_pts_bj - transfo_b_c.t);

    Map<Vector2d> residual(residuals);
    double dep_j = img_pts_camera.z();
    residual = pts_j.head<2>() - (img_pts_camera / dep_j).head<2>();
    residual = sqrt_info * residual;

    if (jacobians)
    {
        Matrix3d R_b_c = transfo_b_c.q.toRotationMatrix();
        Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << -1. / dep_j, 0, img_pts_camera(0) / (dep_j * dep_j),
            0, -1. / dep_j, img_pts_camera(1) / (dep_j * dep_j);

        reduce = sqrt_info * reduce;

        /*****Jacobian derivation*****/
        if (jacobians[0])
        {
            Map<Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

            Matrix<double, 3, 6> jaco_j;

            jaco_j.leftCols<3>() = R_b_c.transpose() * Rj.transpose();
            jaco_j.rightCols<3>() = -R_b_c.transpose() * Utility::skewSymmetric(img_pts_bj);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
    }
    sum_t += tic_toc.toc();

    return true;
}

bool PnpFactor::checkSingularVal(double **parameters)
{
    Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);
    Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Vector3d img_pts_bj = Qj.inverse() * (f_pts_w - Pj);
    Vector3d img_pts_camera = transfo_b_c.q.inverse() * (img_pts_bj - transfo_b_c.t);

    Matrix3d R_b_c = transfo_b_c.q.toRotationMatrix();
    Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix<double, 2, 3> reduce(2, 3);
    double dep_j = img_pts_camera.z();
    reduce << -1. / dep_j, 0, img_pts_camera(0) / (dep_j * dep_j),
        0, -1. / dep_j, img_pts_camera(1) / (dep_j * dep_j);

    Matrix<double, 2, 6> jacobian_pose_j;

    Matrix<double, 3, 6> jaco_j;

    jaco_j.leftCols<3>() = R_b_c.transpose() * Rj.transpose();
    jaco_j.rightCols<3>() = -R_b_c.transpose() * Utility::skewSymmetric(img_pts_bj);

    jacobian_pose_j = reduce * jaco_j;
    JacobiSVD<Matrix<double, 2, 6>> svd(jacobian_pose_j, 0);
    // cout << "singular values are:" << endl << svd.singularValues() << endl;
    Vector2d singular_val = svd.singularValues();
    
    return (singular_val(0) > 1.5);
}