#include "lvio_initialization.h"

/*void solveDeltaQuaternion(map<int, LaserFrame> &all_laser_frame)
{
    Eigen::Matrix4d A;
    Eigen::Vector4d b;
    Eigen::Vector4d delta_theta;
    A.setZero();
    b.setZero();
    map<int, LaserFrame>::iterator frame_i;
    map<int, LaserFrame>::iterator frame_j;
    for (frame_i = all_laser_frame.begin(); std::next(frame_i) != all_laser_frame.end(); frame_i++)
    {
        frame_j = std::next(frame_i);
        Eigen::MatrixXd tmp_A(4,4);
	    tmp_A.setZero();
	    Eigen::VectorXd tmp_b(4);
        tmp_b.setZero();

        Eigen::Quaterniond q_ij(frame_i->second.Tinit2odom_I.q.inverse() * frame_j->second.Tinit2odom_I.q);
        // std::cout << "delta_q:" << frame_j->second.lvio_pre_integration->delta_q.inverse().vec().transpose() << std::endl;
        // std::cout << "q_ij:" << q_ij.vec().transpose() << std::endl;
        Eigen::Quaterniond tmp_q = frame_j->second.lvio_pre_integration->delta_q.inverse()* q_ij;
        tmp_A <<tmp_q.w(), -tmp_q.x(), -tmp_q.y(), -tmp_q.z(),
                tmp_q.x(), tmp_q.w(), -tmp_q.z(), tmp_q.y(),
                tmp_q.y(), tmp_q.z(), tmp_q.w(), -tmp_q.x(), 
                tmp_q.z(), -tmp_q.y(), tmp_q.x(), tmp_q.w();        
        tmp_b << 1, 0, 0, 0;
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_theta = A.ldlt().solve(b);
    // ROS_DEBUG_STREAM("solved b(0): "<< delta_theta(0));
    double solid_angle = 2 * acos(delta_theta(0)) * 180.0 / M_PI;
    ROS_WARN_STREAM("Initial calibration, gyro bias is : "<< solid_angle);

    // if (solid_angle > 3) Need to use lidar to correct?????
}*/

/*bool LvioLinearAlignment(map<int, LaserFrame> &all_laser_frame, Vector3d &g, VectorXd &x)
{
    int all_frame_count = all_laser_frame.size();
    int n_state = all_frame_count * 3 + 3;
    
    Eigen::MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<int, LaserFrame>::iterator frame_i;
    map<int, LaserFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_laser_frame.begin(); next(frame_i) != all_laser_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 9);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.lvio_pre_integration->sum_dt;
        Matrix3d frame_i_w_b = frame_i->second.Tinit2odom_I.q.normalized().toRotationMatrix();
        Matrix3d frame_j_w_b = frame_j->second.Tinit2odom_I.q.normalized().toRotationMatrix();

        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = 0.5 * frame_i_w_b.transpose() * dt * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(0, 0) = frame_j->second.lvio_pre_integration->delta_p - frame_i_w_b.transpose() * (frame_j->second.Tinit2odom_I.v - frame_i->second.Tinit2odom_I.v);

        tmp_A.block<3, 3>(3, 0) = - Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i_w_b.transpose() * frame_j_w_b;
        tmp_A.block<3, 3>(3, 6) = frame_i_w_b.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.lvio_pre_integration->delta_v;
        //cout << "delta_v : " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>(); // 30
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>(); // 4
        b.tail<3>() += r_b.tail<3>();

        A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
        A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b); // 34 * 1
    g = x.segment<3>(n_state - 3);
    ROS_DEBUG_STREAM(" before g     " << g.norm() << " " << g.transpose());

    // refine gravity
    if(fabs(g.norm() - G.norm()) > 1.0)
        return false;
    else 
    {
        g = g * G.norm() / g.norm();
        return true;
    }
}*/

void solveGyroscopeBias(map<int, LaserFrame> &all_laser_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero(3, 3);
    b.setZero(3, 1);
    std::map<int, LaserFrame>::iterator frame_i;
    std::map<int, LaserFrame>::iterator frame_j;
    for (frame_i = all_laser_frame.begin(); std::next(frame_i) != all_laser_frame.end(); frame_i++)
    {
        frame_j = std::next(frame_i);
        Eigen::Matrix3d tmp_A;
        tmp_A.setZero(3, 3);
        Eigen::Vector3d tmp_b;
        tmp_b.setZero(3, 1);

        Eigen::Quaterniond q_ij = frame_i->second.Tinit2odom_I.q.inverse() * frame_j->second.Tinit2odom_I.q;
        // Jacobian is 15 * 15 matrix (O_R, O_BG) = (3, 12)
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_bg = A.ldlt().solve(b);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());
    for (int i = 0; i < SWEEP_SIZE + 1; i++)
        Bgs[i] += delta_bg;
    for (frame_i = all_laser_frame.begin(); std::next(frame_i) != all_laser_frame.end(); frame_i ++){
        frame_j = std::next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

bool LinearAlignment(map<int, LaserFrame> &all_laser_frame, Vector3d &g, VectorXd &x)
{
    int all_frame_count = all_laser_frame.size();
    int n_state = all_frame_count * 3 + 3;
    
    Eigen::MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<int, LaserFrame>::iterator frame_i;
    map<int, LaserFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_laser_frame.begin(); next(frame_i) != all_laser_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 9);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;
        
        Matrix3d frame_i_w_b = frame_i->second.Tinit2odom_I.q.normalized().toRotationMatrix();
        Matrix3d frame_j_w_b = frame_j->second.Tinit2odom_I.q.normalized().toRotationMatrix();

        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = 0.5 * frame_i_w_b.transpose() * dt * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_i_w_b.transpose() * (frame_j->second.Tinit2odom_I.v - frame_i->second.Tinit2odom_I.v);

        tmp_A.block<3, 3>(3, 0) = - Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i_w_b.transpose() * frame_j_w_b;
        tmp_A.block<3, 3>(3, 6) = frame_i_w_b.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v : " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>(); // 30
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>(); // 4
        b.tail<3>() += r_b.tail<3>();

        A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
        A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();

        ROS_DEBUG_STREAM("count: " << frame_i->first);
        ROS_INFO("t: %f",frame_i->second.t);
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b); // 34 * 1
    g = x.segment<3>(n_state - 3);
    ROS_DEBUG_STREAM(" before g" << g.norm() << " " << g.transpose());

    // refine gravity
    if(fabs(g.norm() - G.norm()) > 1.0)
        return false;
    else 
    {
        g = g * G.norm() / g.norm();
        return true;
    }
    ROS_DEBUG_STREAM("frame count:" << all_frame_count);
}

bool LaserImuAlignment(map<int, LaserFrame> &all_laser_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    solveGyroscopeBias(all_laser_frame, Bgs);

    if(LinearAlignment(all_laser_frame, g, x))
        return true;
    else
        return false;
}
