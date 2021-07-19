#include "ncrl_livins/depth_map_estimator.h"
/**
 * @brief:Temporal Calibration
 **/
// #define ADD_DEPTHMAP_TD
// #define ADD_VISION_TD

/**
 * @brief:Odometry
 **/
// #define ADD_IMU
// #define ADD_DEPMAP
#define ADD_VINS_PRJ
// #define ADD_LOAM_F2F_FLAT
#define ADD_F2M_CORNER
#define ADD_F2M_FLAT

DepthMapEstimator::DepthMapEstimator() : pts_manager()
{
    ROS_INFO("init begins");
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    solver_flag = INITIAL;
    first_imu = false,
    frame_count = 0;
    received_num = 0;
}

void DepthMapEstimator::initSpeedBias(VinsType &vins_msg)
{
    if (solver_flag == SolverFlag::INITIAL)
        Vs[frame_count] = Vector3d(vins_msg.first.second->velocity.x,
                                   vins_msg.first.second->velocity.y,
                                   vins_msg.first.second->velocity.z);
    else
    {
        PointUtility::Tf transfo_w_bi_inv = PointUtility::inverse(pts_manager.poses_frames[frame_count - 1]->pose_vins);
        PointUtility::Tf transfo_w_odom = PointUtility::addTransfo(pts_manager.poses_frames[frame_count - 1]->Pose,
                                                                   transfo_w_bi_inv);
        Vector3d velocity_odom_bj(vins_msg.first.second->velocity.x,
                                  vins_msg.first.second->velocity.y,
                                  vins_msg.first.second->velocity.z);

        Vs[frame_count] = transfo_w_odom.q * velocity_odom_bj;
    }

    Bas[frame_count] = Vector3d(vins_msg.first.second->bias_acc.x,
                                vins_msg.first.second->bias_acc.y,
                                vins_msg.first.second->bias_acc.z);
    Bgs[frame_count] = Vector3d(vins_msg.first.second->bias_gyro.x,
                                vins_msg.first.second->bias_gyro.y,
                                vins_msg.first.second->bias_gyro.z);
}

void DepthMapEstimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
#ifdef ADD_IMU
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if (frame_count != 0)
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
#endif //ADD_IMU
}

void DepthMapEstimator::setLidarTransfo(VinsType &first_vins_data,
                                        VinsType &second_vins_data,
                                        double s, int use_index)
{
    Quaterniond q_first(first_vins_data.first.second->orientation.w,
                        first_vins_data.first.second->orientation.x,
                        first_vins_data.first.second->orientation.y,
                        first_vins_data.first.second->orientation.z);

    Vector3d t_first(first_vins_data.first.second->position.x,
                     first_vins_data.first.second->position.y,
                     first_vins_data.first.second->position.z);

    Quaterniond q_second(second_vins_data.first.second->orientation.w,
                         second_vins_data.first.second->orientation.x,
                         second_vins_data.first.second->orientation.y,
                         second_vins_data.first.second->orientation.z);

    Vector3d t_second(second_vins_data.first.second->position.x,
                      second_vins_data.first.second->position.y,
                      second_vins_data.first.second->position.z);

    Quaterniond q_sec_fir = q_second.inverse() * q_first;

    if (use_index == 1)
    {
        pts_manager.transfo_cj_c_near.q = Quaterniond::Identity().slerp(s, q_sec_fir);
        pts_manager.transfo_cj_c_near.t = s * (q_second.inverse() * (t_first - t_second));
    }
    else
    {
        pts_manager.transfo_cj_c_near.q = Quaterniond::Identity().slerp(s, q_sec_fir.inverse());
        pts_manager.transfo_cj_c_near.t = s * (q_first.inverse() * (t_second - t_first));
    }
}

void DepthMapEstimator::processLVIO(VinsType &vins_msg, LaserType &laser_msg)
{
    headers[frame_count] = vins_msg.first.first.first.header;
    img_t[frame_count] = headers[frame_count].stamp.toSec() + vins_msg.first.first.first.cur_td;
    laser_t[frame_count] = laser_msg.second.second.first->header.stamp.toSec();

    f_arrow_l.clear();
    predict_f_pts_cj.clear();
    solved_pts.clear();
    received_num = 0;

    pts_manager.frame_count = frame_count;
    pts_manager.preprocessing(vins_msg, laser_msg);

    if (solver_flag == INITIAL)
    {
        // ROS_DEBUG("0");
        pts_manager.updateGlobalMap();
        solver_flag = NON_LINEAR;
        frame_count++;
    }
    else
    {
        // ROS_DEBUG("1");
        pts_manager.guessTransform();
        pts_manager.prepareLocalmap();
        // ROS_DEBUG("2");
        optimization();
        // ROS_DEBUG("3");
        pts_manager.updateGlobalMap();
        // ROS_DEBUG("4");
        slideWindow();
        // ROS_DEBUG("5");
    }
    pts_manager.sweep_id++;
}

void DepthMapEstimator::vector2double()
{

    // ROS_DEBUG("01");
    if (ESTIMATE_LC_TD)
    {
        para_Td[0][0] = laser_t[frame_count] - img_t[frame_count];
        ROS_DEBUG_STREAM("initial guess td: \t" << para_Td[0][0]);
    }
    else
    {
        // Given initial value of robot poses from VINS
        for (int i = 0; i <= frame_count; i++)
        {
            para_Pose[i][0] = pts_manager.poses_frames[i]->Pose.t.x();
            para_Pose[i][1] = pts_manager.poses_frames[i]->Pose.t.y();
            para_Pose[i][2] = pts_manager.poses_frames[i]->Pose.t.z();
            para_Pose[i][3] = pts_manager.poses_frames[i]->Pose.q.x();
            para_Pose[i][4] = pts_manager.poses_frames[i]->Pose.q.y();
            para_Pose[i][5] = pts_manager.poses_frames[i]->Pose.q.z();
            para_Pose[i][6] = pts_manager.poses_frames[i]->Pose.q.w();
#ifdef ADD_IMU
            if (add_imu)
            {
                para_SpeedBias[i][0] = Vs[i].x();
                para_SpeedBias[i][1] = Vs[i].y();
                para_SpeedBias[i][2] = Vs[i].z();

                para_SpeedBias[i][3] = Bas[i].x();
                para_SpeedBias[i][4] = Bas[i].y();
                para_SpeedBias[i][5] = Bas[i].z();

                para_SpeedBias[i][6] = Bgs[i].x();
                para_SpeedBias[i][7] = Bgs[i].y();
                para_SpeedBias[i][8] = Bgs[i].z();
            }
#endif //ADD_IMU
        }
    }

    // ROS_DEBUG("02");
    predict_f_pts_cj = pts_manager.guessFeature();

    // Input the initial guess of the depth of features from image
    int corres_cnt = 0;
    for (auto &point : pts_manager.pts_frames[frame_count]->image)
    {
        // ROS_DEBUG("03");
        int feature_id = point.first;
        bool plane_sw = 0;
        bool image_sw = 0;

#ifdef ADD_DEPMAP
        auto it = pts_manager.pts_frames[(frame_count - 1)]->flat_pts_l.find(feature_id); // take more time to search
        if (it != pts_manager.pts_frames[(frame_count - 1)]->flat_pts_l.end())
            plane_sw = 1;
#endif //ADD_DEPMAP

#ifdef ADD_VINS_PRJ
        auto f_it = pts_manager.pts_frames[(frame_count - 1)]->image.find(feature_id);
        if (f_it != pts_manager.pts_frames[(frame_count - 1)]->image.end())
        {
            // Check the feature with the initial guess of the depth
            auto pred_it = predict_f_pts_cj.find(feature_id);
            if (pred_it != predict_f_pts_cj.end())
                image_sw = 1;
        }
#endif //ADD_VINS_PRJ

        if (plane_sw || image_sw)
        {
            para_Feature[corres_cnt][0] = 1.0 / predict_f_pts_cj.at(feature_id).z();
            corres_cnt++;
        }
        // ROS_DEBUG("05");
    }
}

void DepthMapEstimator::double2vector()
{
    /**
     * @brief: Keep the consistency
     */
    Quaterniond q0(pts_manager.poses_frames[0]->Pose.q.w(),
                   pts_manager.poses_frames[0]->Pose.q.x(),
                   pts_manager.poses_frames[0]->Pose.q.y(),
                   pts_manager.poses_frames[0]->Pose.q.z());
    Matrix3d R0 = q0.toRotationMatrix();
    Vector3d origin_R0 = Utility::R2ypr(R0);
    Vector3d origin_P0(pts_manager.poses_frames[0]->Pose.t.x(),
                       pts_manager.poses_frames[0]->Pose.t.y(),
                       pts_manager.poses_frames[0]->Pose.t.z());

    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();

    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = R0 * Quaterniond(para_Pose[0][6],
                                    para_Pose[0][3],
                                    para_Pose[0][4],
                                    para_Pose[0][5])
                            .toRotationMatrix()
                            .transpose();
    }

    for (int i = 0; i <= frame_count; i++)
    {
        // Here Qs and Ps are output
        pts_manager.poses_frames[i]->Pose.q = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

        pts_manager.poses_frames[i]->Pose.t = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                                                  para_Pose[i][1] - para_Pose[0][1],
                                                                  para_Pose[i][2] - para_Pose[0][2]) +
                                              origin_P0;
#ifdef ADD_IMU
        if (add_imu)
        {
            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
#endif //ADD_IMU
    }

    recoverDepth();
}

double DepthMapEstimator::evaluateDepth()
{
    double sum_distance = 0;
    int pts_number = pts_manager.pts_frames[frame_count]->flat_pts_l.size();
    // ROS_FATAL_STREAM("point # \t" << pts_number);
    for (auto it : pts_manager.pts_frames[frame_count]->flat_pts_l)
    {
        Eigen::Vector3d nu;
        nu = (it.second[2] - it.second[1]).cross(it.second[2] - it.second[0]);
        nu.normalized();

        double point_plane_distance = nu.dot(it.second[2] - it.second[3]);
        sum_distance += point_plane_distance;
    }

    return (sum_distance / pts_number);
}

void DepthMapEstimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *vision_loss_function, *lidar_loss_function;
    vision_loss_function = new ceres::CauchyLoss(1.0);
    // lidar_loss_function = new ceres::CauchyLoss(0.5);
    lidar_loss_function = new ceres::HuberLoss(0.1);

    add_imu = true;
    // double avg_distance = evaluateDepth();
    // if (avg_distance < 0.05 * DISTANCE_THRES)
    // {
    //     add_imu = true;
    //     ROS_WARN_STREAM("Solve the speed and bias, the avg distance is:\t" << avg_distance);
    // }

    // ROS_DEBUG("11");
    if (ESTIMATE_LC_TD)
        problem.AddParameterBlock(para_Td[0], 1);
    else
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        for (int i = 0; i <= frame_count; i++)
        {
            problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
#ifdef ADD_IMU
            if (add_imu)
                problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
#endif //ADD_IMU
        }
    }

    // ROS_DEBUG("12");
    TicToc t_whole, t_prepare;
    // Give initial guess to the parameter block
    vector2double();

#ifdef ADD_IMU
    if (add_imu)
    {
        if (pre_integrations[frame_count]->sum_dt > 10.0)
        {
        }
        else
        {
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[frame_count]);
            problem.AddResidualBlock(imu_factor, NULL,
                                     para_Pose[frame_count - 1], para_SpeedBias[frame_count - 1],
                                     para_Pose[frame_count], para_SpeedBias[frame_count]);
        }
    }
#endif //ADD_IMU

    // ROS_DEBUG("13");
    int corres_cnt = 0, f_plane_cnt = 0, f_image_cnt = 0, rel_cnt = 0;
    for (auto &point : pts_manager.pts_frames[frame_count]->image)
    {
        int feature_id = point.first;
        bool plane_sw = 0;
        bool image_sw = 0;

        // Input image feature to search the Lidar correspondence of features in the plane
        auto it = pts_manager.pts_frames[(frame_count - 1)]->flat_pts_l.find(feature_id); // take more time to search
        if (it != pts_manager.pts_frames[(frame_count - 1)]->flat_pts_l.end())
        {
            Vector3d pt_j(point.second(0), point.second(1), point.second(2));
            Vector3d pts_l, pts_m, pts_n; // Find the correspondence from the lidar flat points

            pts_l = it->second[0];
            pts_m = it->second[1];
            pts_n = it->second[2];

            // ROS_DEBUG("14");
            if (ESTIMATE_LC_TD)
            {
#ifdef ADD_DEPTHMAP_TD
                DepthMapTdFactor *d_td = new DepthMapTdFactor(pts_manager.poses_frames[frame_count]->transfo_li_lj,
                                                              pts_manager.transfo_l_c[frame_count],
                                                              pt_j, nu, pts_n, velocity_j, LC_TD);

                // Check whether this residual could be put to cost function
                double **para = new double *[2];
                para[0] = para_Feature[corres_cnt];
                para[1] = para_Td[0];
                if (d_td->checkResidual(para) > 0.1)
                {
                    // ROS_WARN("Original residual: %f", d_td->checkResidual(para));
                    Vector3d f_pts_li;
                    f_pts_li = pts_manager.pts_frames[(frame_count - 1)]->f_pts_lj.at(feature_id);

                    vector<Vector3d> debug_set;
                    debug_set.reserve(4);
                    debug_set.push_back(f_pts_li);
                    debug_set.push_back(pts_l);
                    debug_set.push_back(pts_m);
                    debug_set.push_back(pts_n);
                    f_arrow_l.push_back(debug_set);

                    fail_cnt++;
                }

                problem.AddResidualBlock(d_td, loss_function, para_Feature[corres_cnt], para_Td[0]);
                d_td->check(para);
#endif

#ifdef ADD_VISION_TD
                // Add the residual of the 3D-2D vision
                Vector3d f_pts_li;
                f_pts_li = pts_manager.pts_frames[(frame_count - 1)]->f_pts_lj.at(feature_id);

                PnpTdFactor *p_td = new PnpTdFactor(pts_manager.poses_frames[frame_count]->transfo_li_lj,
                                                    pts_manager.transfo_l_c[frame_count],
                                                    pt_j, f_pts_li, velocity_j, LC_TD);
                problem.AddResidualBlock(p_td, vision_loss_function, para_Feature[corres_cnt], para_Td[0]);

                // double **para = new double *[2];
                // para[0] = para_Feature[correspondence_cnt];
                // para[1] = para_Td[0];
                // p_td->check(para);
#endif
            }
            else
            {
#ifdef ADD_DEPMAP
                DepthMapFactor *depMap_factor = new DepthMapFactor(pts_l, pts_m, pts_n, pt_j,
                                                                   pts_manager.transfo_b_l,
                                                                   pts_manager.transfo_b_c[frame_count]);
                problem.AddResidualBlock(depMap_factor, vision_loss_function, para_Pose[frame_count - 1], para_Pose[frame_count], para_Feature[corres_cnt]);
#endif //ADD_DEPMAP
            }

            plane_sw = 1;
            f_plane_cnt++;
        }

        if (0)
        {
            auto f_it = pts_manager.pts_frames[frame_count]->f_pts_w.find(feature_id);
            if (f_it != pts_manager.pts_frames[frame_count]->f_pts_w.end())
            {
                auto pred_it = predict_f_pts_cj.find(feature_id);
                if (pred_it != predict_f_pts_cj.end())
                {
                    Vector3d pt_w(f_it->second(0), f_it->second(1), f_it->second(2));
                    Vector3d pt_j(point.second(0), point.second(1), point.second(2));
                    double inv_dep = 1.0 / pred_it->second(2);

                    PnpFactor *pnp_factor = new PnpFactor(pts_manager.transfo_b_c[frame_count],
                                                          pt_j, pt_w, inv_dep);
                    double **para = new double *[1];
                    para[0] = para_Pose[frame_count];

                    if (pnp_factor->checkSingularVal(para))
                        received_num++;
                    problem.AddResidualBlock(pnp_factor, vision_loss_function, para_Pose[frame_count]);
                    rel_cnt++;
                }
            }
        }

#ifdef ADD_VINS_PRJ
        if (!ESTIMATE_LC_TD)
        {
            // Input image feature to search the image correspondence of features
            auto f_it = pts_manager.pts_frames[(frame_count - 1)]->image.find(feature_id);
            if (f_it != pts_manager.pts_frames[(frame_count - 1)]->image.end())
            {
                // Check the feature with the initial guess of the depth
                auto pred_it = predict_f_pts_cj.find(feature_id);
                if (pred_it != predict_f_pts_cj.end())
                {
                    Eigen::Vector3d pt_i(f_it->second(0), f_it->second(1), f_it->second(2));
                    Eigen::Vector3d pt_j(point.second(0), point.second(1), point.second(2));

                    ProjectionFactor *img_factor = new ProjectionFactor(pt_i, pt_j, pts_manager.transfo_b_c[frame_count]);
                    problem.AddResidualBlock(img_factor, vision_loss_function, para_Pose[frame_count - 1], para_Pose[frame_count], para_Feature[corres_cnt]);
                    // ProjectionFactor *img_factor = new ProjectionFactor(pt_i, pt_j, pts_manager.transfo_b_c[frame_count], para_Feature[corres_cnt][0]);
                    // problem.AddResidualBlock(img_factor, vision_loss_function, para_Pose[frame_count - 1], para_Pose[frame_count]);

                    image_sw = 1;
                    f_image_cnt++;
                }
            }
        }
#endif //ADD_VINS_PRJ

        if (plane_sw || image_sw)
            corres_cnt++;
    }

#ifdef ADD_LOAM_F2F_FLAT
    // pts_manager.downsampling(pts_manager.pts_frames[frame_count]->laser_cloud_flat, pts_manager.pts_frames[frame_count]->laser_cloud_flat, planeRes);
    pts_manager.f2f_search_flat->setInputCloud(pts_manager.pts_frames[frame_count - 1]->laser_cloud_flat);
    int pts_current_num = pts_manager.pts_frames[frame_count]->laser_cloud_flat->size();
    for (int i = 0; i < pts_current_num; i++)
    {
        // Transform to last frame
        Vector3d lidar_cur_pt(pts_manager.pts_frames[frame_count]->laser_cloud_flat->points[i].x,
                              pts_manager.pts_frames[frame_count]->laser_cloud_flat->points[i].y,
                              pts_manager.pts_frames[frame_count]->laser_cloud_flat->points[i].z);

        Vector3d cur_pt_last(0, 0, 0);
        // If lidar time align the image time
        // cur_pt_last = pts_manager.transfo_b_l.q * lidar_cur_pt + pts_manager.transfo_b_l.t;
        // cur_pt_last = pts_manager.poses_frames[frame_count]->Pose.q * cur_pt_last + pts_manager.poses_frames[frame_count]->Pose.t;
        // cur_pt_last = pts_manager.poses_frames[frame_count - 1]->Pose.q.inverse() * (cur_pt_last - pts_manager.poses_frames[frame_count - 1]->Pose.t);
        // cur_pt_last = pts_manager.transfo_b_l.q.inverse() * (cur_pt_last - pts_manager.transfo_b_l.t);

        // If lidar time don't align the image time, the correspondence need to be found.
        PointUtility::Tf transfo_li_lj = PointUtility::findF2FTransfo(pts_manager.poses_frames[(frame_count - 1)]->pose_aloam,
                                                                      pts_manager.poses_frames[frame_count]->pose_aloam);
        cur_pt_last = transfo_li_lj.q * lidar_cur_pt + transfo_li_lj.t;

        PointType point;
        point.x = cur_pt_last.x();
        point.y = cur_pt_last.y();
        point.z = cur_pt_last.z();

        // kd-tree search
        vector<Vector3d> pts_found = pts_manager.getF2FflatCorres(point);
        if (!pts_found.empty())
        {
            LidarPlaneFactor *f2f_plane_factor = new LidarPlaneFactor(lidar_cur_pt, pts_found[0],
                                                                      pts_found[1], pts_found[2],
                                                                      pts_manager.transfo_b_l, 1);
            problem.AddResidualBlock(f2f_plane_factor, lidar_loss_function, para_Pose[frame_count - 1], para_Pose[frame_count]);
        }
    }
#endif //ADD_LOAM_F2F_PLANE

    ROS_DEBUG_STREAM("frame count:" << frame_count);
    for (int i = frame_count; (i > (frame_count - 2)) && (i > -1); --i)
    {
        ROS_DEBUG_STREAM("id:" << i);
        int local_map_corner_pts_num = pts_manager.pts_frames[i]->local_map_corner->points.size();
#ifdef ADD_F2M_CORNER
        int sum_c = 0;
        if (local_map_corner_pts_num > 10)
        {
            TicToc t_tree;
            pts_manager.f2m_search_corner->setInputCloud(pts_manager.pts_frames[i]->local_map_corner);
            ROS_DEBUG_STREAM("Build kdtree time for coner points: " << t_tree.toc());

            PtCloud laser_cloud_corner_downsampled(new pcl::PointCloud<PointType>());
            pts_manager.downsampling(pts_manager.pts_frames[i]->laser_cloud_corner,
                                     laser_cloud_corner_downsampled, lineRes);
            int pts_current_num = laser_cloud_corner_downsampled->points.size();
            for (int j = 0; j < pts_current_num; j++)
            {
                Vector3d lidar_cur_pt(laser_cloud_corner_downsampled->points[j].x,
                                      laser_cloud_corner_downsampled->points[j].y,
                                      laser_cloud_corner_downsampled->points[j].z);

                Vector3d cur_pt_world(0, 0, 0);
                cur_pt_world = pts_manager.transfo_b_l.q * lidar_cur_pt + pts_manager.transfo_b_l.t;
                cur_pt_world = pts_manager.poses_frames[i]->Pose.q * cur_pt_world + pts_manager.poses_frames[i]->Pose.t;

                PointType point;
                point.x = cur_pt_world.x();
                point.y = cur_pt_world.y();
                point.z = cur_pt_world.z();

                // kd-tree search, and get line point, point_a, point_b
                vector<Vector3d> pts_found = pts_manager.getF2McornerCorres(point, i);
                if (!pts_found.empty())
                {
                    LidarEdgeNormFactor *f2m_edge_factor = new LidarEdgeNormFactor(lidar_cur_pt,
                                                                                   pts_found[0], pts_found[1],
                                                                                   pts_manager.transfo_b_l, 5);
                    problem.AddResidualBlock(f2m_edge_factor, lidar_loss_function, para_Pose[i]);
                    sum_c++;
                }
            }
            ROS_DEBUG_STREAM("Edge correspondence number in optimization:" << sum_c);
        }
#endif //ADD_F2M_CORNER

#ifdef ADD_F2M_FLAT
        int sum_f = 0;
        int local_map_flat_pts_num = pts_manager.pts_frames[i]->local_map_flat->points.size();
        if (local_map_flat_pts_num > 50)
        {
            TicToc t_tree;
            pts_manager.f2m_search_flat->setInputCloud(pts_manager.pts_frames[i]->local_map_flat);
            ROS_DEBUG_STREAM("Build kdtree time for flat points: " << t_tree.toc());

            PtCloud laser_cloud_flat_downsampled(new pcl::PointCloud<PointType>());
            pts_manager.downsampling(pts_manager.pts_frames[i]->laser_cloud_flat,
                                     laser_cloud_flat_downsampled, planeRes);
            int pts_current_num = laser_cloud_flat_downsampled->points.size();
            ROS_DEBUG_STREAM("flat points input number:" << pts_current_num);
            for (int j = 0; j < pts_current_num; j++)
            {
                Vector3d lidar_cur_pt(laser_cloud_flat_downsampled->points[j].x,
                                      laser_cloud_flat_downsampled->points[j].y,
                                      laser_cloud_flat_downsampled->points[j].z);

                Vector3d cur_pt_world(0, 0, 0);
                cur_pt_world = pts_manager.transfo_b_l.q * lidar_cur_pt + pts_manager.transfo_b_l.t;
                cur_pt_world = pts_manager.poses_frames[i]->Pose.q * cur_pt_world + pts_manager.poses_frames[i]->Pose.t;

                PointType point;
                point.x = cur_pt_world.x();
                point.y = cur_pt_world.y();
                point.z = cur_pt_world.z();

                // kd-tree search, and get the normal vector of the plane
                VectorXd norm_valid = pts_manager.getF2MflatCorres(point, i);
                if (norm_valid.size() == 4)
                {
                    Vector3d nu = norm_valid.head<3>();
                    // ROS_DEBUG_STREAM("nu check:" << nu);
                    double negative_OA_dot_nu = norm_valid(3);

                    LidarPlaneNormFactor *f2m_plane_factor = new LidarPlaneNormFactor(lidar_cur_pt,
                                                                                      nu, negative_OA_dot_nu,
                                                                                      pts_manager.transfo_b_l, 10);
                    problem.AddResidualBlock(f2m_plane_factor, lidar_loss_function, para_Pose[i]);
                    sum_f++;
                }
            }
            ROS_DEBUG_STREAM("Plane correspondence number in optimization:" << sum_f);
        }
#endif //ADD_F2M_FLAT
    }
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());
    ROS_DEBUG_STREAM("number of correspondence:" << corres_cnt);
    ROS_DEBUG_STREAM("number of features in lidar plane:" << f_plane_cnt
                                                          << "\t number of features in image:"
                                                          << f_image_cnt);
    ROS_DEBUG_STREAM("number of correspondence PnP:" << rel_cnt);
    // ROS_FATAL_STREAM("OK PnP corres num:" << received_num);

    ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_QR;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 2;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    options.max_num_iterations = 4;            // default = 8
    options.max_solver_time_in_seconds = 0.04; // default = 0.04
    options.minimizer_progress_to_stdout = false;

    // Debug for optimization
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    ROS_INFO_COND(ESTIMATE_LC_TD, "td: %f", para_Td[0][0]);

    double2vector();

    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void DepthMapEstimator::recoverDepth()
{
    // Debug for the depth of feature
    int corres_cnt = 0;
    for (auto &point : pts_manager.pts_frames[frame_count]->image)
    {
        int feature_id = point.first;
        bool plane_sw = 0;
        bool image_sw = 0;

#ifdef ADD_DEPMAP
        auto it = pts_manager.pts_frames[(frame_count - 1)]->flat_pts_l.find(feature_id);
        if (it != pts_manager.pts_frames[(frame_count - 1)]->flat_pts_l.end())
            plane_sw = 1;
#endif //ADD_DEPMAP

#ifdef ADD_VINS_PRJ
        auto f_it = pts_manager.pts_frames[(frame_count - 1)]->image.find(feature_id);
        if (f_it != pts_manager.pts_frames[(frame_count - 1)]->image.end())
        {
            // Check the feature with the initial guess of the depth
            auto pred_it = predict_f_pts_cj.find(feature_id);
            if (pred_it != predict_f_pts_cj.end())
                image_sw = 1;
        }
#endif //ADD_VINS_PRJ

        if (plane_sw || image_sw)
        {
            Vector3d img_pts_cj(point.second(0), point.second(1), point.second(2));

            if (ESTIMATE_LC_TD)
            {
                Vector3d velocity_j(point.second(5), point.second(6), 0);
                img_pts_cj = img_pts_cj - (para_Td[0][0] - LC_TD) * velocity_j;
            }

            img_pts_cj = img_pts_cj / para_Feature[corres_cnt][0];
            img_pts_cj = pts_manager.transfo_l_c[frame_count].q * img_pts_cj + pts_manager.transfo_l_c[frame_count].t;
            solved_pts[feature_id] = img_pts_cj;

            corres_cnt++;
        }
    }
}

void DepthMapEstimator::slideWindow()
{
    if (frame_count == WINDOW_SIZE)
    {
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            img_t[i] = img_t[i + 1];
            laser_t[i] = laser_t[i + 1];
            headers[i] = headers[i + 1];
#ifdef ADD_IMU
            Vs[i].swap(Vs[i + 1]);
            Bas[i].swap(Bas[i + 1]);
            Bgs[i].swap(Bgs[i + 1]);
#endif //ADD_IMU

            swap(pre_integrations[i], pre_integrations[i + 1]);
        }
        img_t[WINDOW_SIZE] = img_t[WINDOW_SIZE - 1];
        laser_t[WINDOW_SIZE] = laser_t[WINDOW_SIZE - 1];
        headers[WINDOW_SIZE] = headers[WINDOW_SIZE - 1];

#ifdef ADD_IMU
        Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
        Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
        Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

        delete pre_integrations[WINDOW_SIZE];
        pre_integrations[WINDOW_SIZE] = NULL;
        // pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
#endif //ADD_IMU

        pts_manager.shiftPtsWindow();
    }
    else
        frame_count++;
}