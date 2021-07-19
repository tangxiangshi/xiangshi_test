#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include "../utility/utility.h"
#include "../utility/point_utility.h"

class LidarEdgeIJFactor : public ceres::SizedCostFunction<3, 7, 7, 7>
{
public:
    LidarEdgeIJFactor() = delete;
    LidarEdgeIJFactor(Eigen::Vector3d curr_point_,
                      Eigen::Vector3d last_point_a_,
                      Eigen::Vector3d last_point_b_,
                      double sqrt_info_) : curr_point(curr_point_),
                                           last_point_a(last_point_a_),
                                           last_point_b(last_point_b_),
                                           sqrt_info(sqrt_info_) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d cp(curr_point.x(), curr_point.y(), curr_point.z());
        Eigen::Vector3d lpa(last_point_a.x(), last_point_a.y(), last_point_a.z());
        Eigen::Vector3d lpb(last_point_b.x(), last_point_b.y(), last_point_b.z());

        //extrinsic define from imu to lidar frame
        Eigen::Vector3d til(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qil(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
        Eigen::Matrix3d ril = qil.toRotationMatrix();

        //transform point from current lidar to current body frame
        Eigen::Vector3d point_j = qil * cp + til;
        //transform point from current body to world frame
        Eigen::Vector3d point_w = Qj * point_j + Pj;
        //transform point from world to last body frame
        Eigen::Vector3d point_i = Qi.inverse() * (point_w - Pi);
        //transform point from last body to last lidar frame
        Eigen::Vector3d lp = qil.inverse() * (point_i - til);

        Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
        Eigen::Vector3d de = lpa - lpb;

        residuals[0] = sqrt_info * nu.x() / de.norm();
        residuals[1] = sqrt_info * nu.y() / de.norm();
        residuals[2] = sqrt_info * nu.z() / de.norm();

        if (jacobians)
        {
            Eigen::Matrix3d Ri = Qi.toRotationMatrix();
            Eigen::Matrix3d Rj = Qj.toRotationMatrix();

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

                Eigen::Matrix3d tran = Utility::skewSymmetric(de) / de.norm() * ril.transpose() * Ri.transpose();
                Eigen::Matrix3d rot = -Utility::skewSymmetric(de) / de.norm() * ril.transpose() *
                                      Utility::skewSymmetric(Ri.transpose() * (Rj * (ril * cp + til) + Pj - Pi));

                jaco_x.leftCols<3>() = tran.block<1, 3>(0, 0);
                jaco_y.leftCols<3>() = tran.block<1, 3>(1, 0);
                jaco_z.leftCols<3>() = tran.block<1, 3>(2, 0);
                jaco_x.rightCols<3>() = rot.block<1, 3>(0, 0);
                jaco_y.rightCols<3>() = rot.block<1, 3>(1, 0);
                jaco_z.rightCols<3>() = rot.block<1, 3>(2, 0);

                jacobian_pose_i.setZero();
                jacobian_pose_i.block<1, 6>(0, 0) = sqrt_info * jaco_x;
                jacobian_pose_i.block<1, 6>(1, 0) = sqrt_info * jaco_y;
                jacobian_pose_i.block<1, 6>(2, 0) = sqrt_info * jaco_z;
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
                Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

                Eigen::Matrix3d tran = -Utility::skewSymmetric(de) / de.norm() * ril.transpose() * Ri.transpose();
                Eigen::Matrix3d rot = Utility::skewSymmetric(de) / de.norm() * ril.transpose() *
                                      Ri.transpose() * Rj * Utility::skewSymmetric(ril * cp + til);

                jaco_x.leftCols<3>() = tran.block<1, 3>(0, 0);
                jaco_y.leftCols<3>() = tran.block<1, 3>(1, 0);
                jaco_z.leftCols<3>() = tran.block<1, 3>(2, 0);
                jaco_x.rightCols<3>() = rot.block<1, 3>(0, 0);
                jaco_y.rightCols<3>() = rot.block<1, 3>(1, 0);
                jaco_z.rightCols<3>() = rot.block<1, 3>(2, 0);

                jacobian_pose_j.setZero();
                jacobian_pose_j.block<1, 6>(0, 0) = sqrt_info * jaco_x;
                jacobian_pose_j.block<1, 6>(1, 0) = sqrt_info * jaco_y;
                jacobian_pose_j.block<1, 6>(2, 0) = sqrt_info * jaco_z;
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_ex(jacobians[2]);
                Eigen::Matrix3d I3x3;
                I3x3.setIdentity();
                Eigen::Matrix<double, 1, 6> jaco_x, jaco_y, jaco_z;

                Eigen::Matrix3d tran = -Utility::skewSymmetric(de) / de.norm() * ril.transpose() * (Ri.transpose() * Rj - I3x3);
                Eigen::Matrix3d rot = Utility::skewSymmetric(de) / de.norm() * ril.transpose() * Ri.transpose() *
                                          Rj * ril * Utility::skewSymmetric(cp) -
                                      Utility::skewSymmetric(de) / de.norm() * Utility::skewSymmetric(ril.transpose() * (Ri.transpose() * (Rj * (ril * cp + til) + Pj - Pi) - til));

                jaco_x.leftCols<3>() = tran.block<1, 3>(0, 0);
                jaco_y.leftCols<3>() = tran.block<1, 3>(1, 0);
                jaco_z.leftCols<3>() = tran.block<1, 3>(2, 0);
                jaco_x.rightCols<3>() = rot.block<1, 3>(0, 0);
                jaco_y.rightCols<3>() = rot.block<1, 3>(1, 0);
                jaco_z.rightCols<3>() = rot.block<1, 3>(2, 0);

                jacobian_ex.setZero();
                jacobian_ex.block<1, 6>(0, 0) = sqrt_info * jaco_x;
                jacobian_ex.block<1, 6>(1, 0) = sqrt_info * jaco_y;
                jacobian_ex.block<1, 6>(2, 0) = sqrt_info * jaco_z;
            }
        }

        return true;
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double sqrt_info;
};

class LidarEdgeNormFactor : public ceres::SizedCostFunction<3, 7>
{
public:
    LidarEdgeNormFactor() = delete;
    LidarEdgeNormFactor(Eigen::Vector3d curr_point_,
                        Eigen::Vector3d last_point_a_,
                        Eigen::Vector3d last_point_b_,
                        double sqrt_info_) : curr_point(curr_point_),
                                             last_point_a(last_point_a_),
                                             last_point_b(last_point_b_),
                                             sqrt_info(sqrt_info_)
    {
        nu = last_point_b - last_point_a;
        nu.normalized();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);

        //transform point from current body to world frame
        Eigen::Vector3d point_w = Qj * curr_point + Pj;

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual = sqrt_info * (last_point_b - point_w).cross(nu);

        if (jacobians)
        {
            Eigen::Matrix3d Rj = Qj.toRotationMatrix();
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

                Eigen::Matrix<double, 3, 6> jaco_i;
                jaco_i.leftCols<3>() = -Utility::skewSymmetric(nu);
                jaco_i.rightCols<3>() = Utility::skewSymmetric(nu) * Rj * Utility::skewSymmetric(curr_point);

                jacobian_pose_j.setZero();
                jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_i;
                jacobian_pose_j.rightCols<1>().setZero();
            }
        }

        return true;
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    Eigen::Vector3d nu;
    double sqrt_info;
};

class LidarPlaneFactor : public ceres::SizedCostFunction<1, 7, 7>
{
public:
    LidarPlaneFactor() = delete;
    LidarPlaneFactor(Eigen::Vector3d curr_point_,
                     Eigen::Vector3d last_point_l_,
                     Eigen::Vector3d last_point_m_,
                     Eigen::Vector3d last_point_n_,
                     PointUtility::Tf transfo_b_l_,
                     double sqrt_info_) : curr_point(curr_point_),
                                          last_point_l(last_point_l_),
                                          last_point_m(last_point_m_),
                                          last_point_n(last_point_n_),
                                          transfo_b_l(transfo_b_l_),
                                          sqrt_info(sqrt_info_)
    {
        nu = (last_point_n - last_point_m).cross(last_point_n - last_point_l);
        nu.normalize();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Matrix3d R_b_l = transfo_b_l.q.toRotationMatrix();

        //transform point from current lidar to current body frame
        Eigen::Vector3d point_j = transfo_b_l.q * curr_point + transfo_b_l.t;
        //transform point from current body to world frame
        Eigen::Vector3d point_w = Qj * point_j + Pj;
        //transform point from world to last body frame
        Eigen::Vector3d point_i = Qi.inverse() * (point_w - Pi);
        //transform point from last body to last lidar frame
        Eigen::Vector3d point_l = transfo_b_l.q.inverse() * (point_i - transfo_b_l.t);
        residuals[0] = sqrt_info * nu.dot(last_point_n - point_l);

        if (jacobians)
        {
            Eigen::Matrix3d Ri = Qi.toRotationMatrix();
            Eigen::Matrix3d Rj = Qj.toRotationMatrix();

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_i;

                jaco_i.leftCols<3>() = nu.transpose() * R_b_l.transpose() * Ri.transpose();
                jaco_i.rightCols<3>() = -nu.transpose() * R_b_l.transpose() * Utility::skewSymmetric(point_i);

                jacobian_pose_i.setZero();
                jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
                jacobian_pose_i.rightCols<1>().setZero();
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
                Eigen::Matrix<double, 1, 6> jaco_j;

                jaco_j.leftCols<3>() = -nu.transpose() * R_b_l.transpose() * Ri.transpose();
                jaco_j.rightCols<3>() = nu.transpose() * R_b_l.transpose() * Ri.transpose() * Rj * Utility::skewSymmetric(point_j);

                jacobian_pose_j.setZero();
                jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_j;
                jacobian_pose_j.rightCols<1>().setZero();
            }
        }

        return true;
    }

    Eigen::Vector3d curr_point, last_point_l, last_point_m, last_point_n;
    Eigen::Vector3d nu;
    PointUtility::Tf transfo_b_l;
    double sqrt_info;
};

class LidarPlaneNormFactor : public ceres::SizedCostFunction<1, 7>
{
public:
    LidarPlaneNormFactor() = delete;
    LidarPlaneNormFactor(Eigen::Vector3d curr_point_,
                         Eigen::Vector3d plane_unit_norm_,
                         double negative_OA_dot_norm_,
                         double sqrt_info_) : curr_point(curr_point_),
                                              plane_unit_norm(plane_unit_norm_),
                                              negative_OA_dot_norm(negative_OA_dot_norm_),
                                              sqrt_info(sqrt_info_) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d t_w_curr(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q_w_curr(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        //transform point from current body to world frame
        Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;

        double residual = plane_unit_norm.dot(point_w) + negative_OA_dot_norm;
        residuals[0] = sqrt_info * residual;

        if (jacobians)
        {
            Eigen::Matrix3d Rj = q_w_curr.toRotationMatrix();

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_j;

                jaco_j.leftCols<3>() = plane_unit_norm.transpose();
                jaco_j.rightCols<3>() = -plane_unit_norm.transpose() * Rj * Utility::skewSymmetric(curr_point);

                jacobian_pose_j.setZero();
                jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_j;
                jacobian_pose_j.rightCols<1>().setZero();
            }
        }

        return true;
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
    PointUtility::Tf transfo_b_l;
    double sqrt_info;
};
