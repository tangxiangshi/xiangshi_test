#include "ncrl_livins/point_manager.h"

// #define ALOAM_GUESS

PointPerFrame::PointPerFrame(const lvio_ros_msgs::PointCloud3 img,
                             const sensor_msgs::PointCloudConstPtr vision_cloud,
                             const LaserType laser_msg) : laser_cloud_corner(new pcl::PointCloud<PointType>()),
                                                          laser_cloud_flat(new pcl::PointCloud<PointType>()),
                                                          laser_cloud_full(new pcl::PointCloud<PointType>()),
                                                          local_map_corner(new pcl::PointCloud<PointType>()),
                                                          local_map_flat(new pcl::PointCloud<PointType>())
{
    inputImage(img);
    inputVinsPtCloud(vision_cloud);
    pcl::fromROSMsg(*laser_msg.second.second.first, *laser_cloud_corner);
    pcl::fromROSMsg(*laser_msg.second.second.second, *laser_cloud_flat);
    pcl::fromROSMsg(*laser_msg.second.first, *laser_cloud_full);
}

void PointPerFrame::inputImage(const lvio_ros_msgs::PointCloud3 img)
{
    for (unsigned int i = 0; i < img.points.size(); i++)
    {
        // int v = first_img.channels[0].values[i] + 0.5;
        // int feature_id = v / NUM_OF_CAM;// NUM_OF_CAM = 1
        int feature_id = img.channels[0].values[i] + 0.5;
        double x = img.points[i].x;
        double y = img.points[i].y;
        double z = img.points[i].z;
        double p_u = img.channels[1].values[i];
        double p_v = img.channels[2].values[i];
        double velocity_x = img.channels[3].values[i];
        double velocity_y = img.channels[4].values[i];
        ROS_ASSERT(z == 1);
        Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        image.insert(pair<int, Matrix<double, 7, 1>>(feature_id, xyz_uv_velocity));
    }
}

void PointPerFrame::inputVinsPtCloud(const sensor_msgs::PointCloudConstPtr vision_cloud)
{
    for (unsigned i = 0; i < vision_cloud->points.size(); i++)
    {
        double feature_id = vision_cloud->channels[0].values[i];
        Vector3d point;
        point << vision_cloud->points[i].x,
            vision_cloud->points[i].y,
            vision_cloud->points[i].z;
        f_pts_w.insert(pair<int, Vector3d>(feature_id, point));
    }
}

FramePose::FramePose(const lvio_ros_msgs::CorrectDataConstPtr vins_pose_speed, const nav_msgs::OdometryConstPtr &lidar_pose)
{
    time = lidar_pose->header.stamp.toSec();

    // VINS pose
    pose_vins.q = Quaterniond(vins_pose_speed->orientation.w,
                              vins_pose_speed->orientation.x,
                              vins_pose_speed->orientation.y,
                              vins_pose_speed->orientation.z);
    pose_vins.t = Vector3d(vins_pose_speed->position.x,
                           vins_pose_speed->position.y,
                           vins_pose_speed->position.z);

    // ALOAM pose
    pose_aloam.q = Quaterniond(lidar_pose->pose.pose.orientation.w,
                               lidar_pose->pose.pose.orientation.x,
                               lidar_pose->pose.pose.orientation.y,
                               lidar_pose->pose.pose.orientation.z);
    pose_aloam.t = Vector3d(lidar_pose->pose.pose.position.x,
                            lidar_pose->pose.pose.position.y,
                            lidar_pose->pose.pose.position.z);

#ifdef ALOAM_GUESS
    Pose = pose_aloam;
#else
    // Predict pose for consistency
    Pose = pose_vins;
#endif //ALOAM_GUESS
}

PointManager::PointManager() : keyframe_position(new pcl::PointCloud<PointType>()),
                               keyframe_time_poses(new pcl::PointCloud<PointXYZIT>()),
                               vins_cloud_filtered(new pcl::PointCloud<PointType>()),
                               keyframes_search(new pcl::KdTreeFLANN<PointType>()),
                               depth_search_by_flat(new pcl::KdTreeFLANN<PointType>()),
                               f2f_search_flat(new pcl::KdTreeFLANN<PointType>()),
                               f2m_search_corner(new pcl::KdTreeFLANN<PointType>()),
                               f2m_search_flat(new pcl::KdTreeFLANN<PointType>())
{
    frame_count = 0;
    sweep_id = 0;

    laserCloudCorner_vec.reserve(4851);
    laserCloudSurface_vec.reserve(4851);

    for (int i = 0; i < 4851; i++)
    {
        laserCloudCorner_vec[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurface_vec[i].reset(new pcl::PointCloud<PointType>());
    }
}

void PointManager::exParamInit(const VinsType &vins_msg)
{
    // Initialize vio extrinsic parameter
    transfo_b_c[frame_count].q.w() = vins_msg.second.first->pose.pose.orientation.w;
    transfo_b_c[frame_count].q.x() = vins_msg.second.first->pose.pose.orientation.x;
    transfo_b_c[frame_count].q.y() = vins_msg.second.first->pose.pose.orientation.y;
    transfo_b_c[frame_count].q.z() = vins_msg.second.first->pose.pose.orientation.z;

    transfo_b_c[frame_count].t.x() = vins_msg.second.first->pose.pose.position.x;
    transfo_b_c[frame_count].t.y() = vins_msg.second.first->pose.pose.position.y;
    transfo_b_c[frame_count].t.z() = vins_msg.second.first->pose.pose.position.z;

    // Given the extrinsic parameter between IMU and Lidar
    transfo_b_l.q = ncrl_tf::Euler2Q(ER_I2L);
    transfo_b_l.t = ET_I2L;

    // Given the extrinsic parameter between Lidar and camera
    transfo_l_c[frame_count] = PointUtility::findF2FTransfo(transfo_b_l, transfo_b_c[frame_count]);
}

void PointManager::alignTransfo(const PointUtility::Tf transfo_cj_c_near)
{
    /**
     * @brief: --------|-------------|-----|-----------------
     *              first(i)       near  second(j)
     */
    PointUtility::Tf transfo_lj_l_near;

    transfo_lj_l_near.q = transfo_cj_c_near.q * transfo_l_c[frame_count].q.inverse();
    transfo_lj_l_near.t = transfo_cj_c_near.q * transfo_l_c[frame_count].q * -transfo_l_c[frame_count].t + transfo_cj_c_near.t;
    transfo_lj_l_near = PointUtility::addTransfo(transfo_l_c[frame_count], transfo_lj_l_near);

    PointUtility::transfoCloud(transfo_lj_l_near,
                               pts_frames[frame_count]->laser_cloud_corner,
                               pts_frames[frame_count]->laser_cloud_corner);

    PointUtility::transfoCloud(transfo_lj_l_near,
                               pts_frames[frame_count]->laser_cloud_flat,
                               pts_frames[frame_count]->laser_cloud_flat);

    PointUtility::transfoCloud(transfo_lj_l_near,
                               pts_frames[frame_count]->laser_cloud_full,
                               pts_frames[frame_count]->laser_cloud_full);
}

void PointManager::depthAssociation(const PtCloud &laser_cloud_flat, const PtCloud &vins_cloud)
{
    // kdtree for depth association
    depth_search_by_flat->setInputCloud(laser_cloud_flat);

    int fail_cnt = 0;
    int pts_num_vision = vins_cloud->points.size();
    vector<int> pt_ids;
    vector<float> pt_search_sq_distance;
    for (int i = 0; i < pts_num_vision; i++)
    {
        depth_search_by_flat->nearestKSearch(vins_cloud->points[i], 1, pt_ids, pt_search_sq_distance);

        // Prevent lidar points to lie on the same line
        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        if (pt_search_sq_distance[0] < SQ_NORM_THRES)
        {
            closestPointInd = pt_ids[0];

            int closestPointScanID = int(laser_cloud_flat->points[closestPointInd].intensity);
            double minPointSqDis2 = SQ_NORM_THRES, minPointSqDis3 = SQ_NORM_THRES;

            for (int j = closestPointInd + 1; j < (int)laser_cloud_flat->points.size(); j++)
            {
                // if upper scan
                if (int(laser_cloud_flat->points[j].intensity) > (closestPointInd + NEARBY_SCAN))
                    break;

                double pointSqDis = pow((laser_cloud_flat->points[j].x - vins_cloud->points[i].x), 2) +
                                    pow((laser_cloud_flat->points[j].y - vins_cloud->points[i].y), 2) +
                                    pow((laser_cloud_flat->points[j].z - vins_cloud->points[i].z), 2);

                // m-point with same scan line as l-point
                if (int(laser_cloud_flat->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                {
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                }
                else if (int(laser_cloud_flat->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                {
                    minPointSqDis3 = pointSqDis;
                    minPointInd3 = j;
                }
            }

            for (int j = closestPointInd - 1; j >= 0; j--)
            {
                if (int(laser_cloud_flat->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                    break;

                double pointSqDis = pow((laser_cloud_flat->points[j].x - vins_cloud->points[i].x), 2) +
                                    pow((laser_cloud_flat->points[j].y - vins_cloud->points[i].y), 2) +
                                    pow((laser_cloud_flat->points[j].z - vins_cloud->points[i].z), 2);

                if (int(laser_cloud_flat->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                {
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                }
                else if (int(laser_cloud_flat->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                {
                    minPointSqDis3 = pointSqDis;
                    minPointInd3 = j;
                }
            }
        }

        if (minPointInd2 >= 0 && minPointInd3 >= 0)
        {
            Vector3d vision_point(vins_cloud->points[i].x,
                                  vins_cloud->points[i].y,
                                  vins_cloud->points[i].z);
            Vector3d lidar_point_l(laser_cloud_flat->points[closestPointInd].x,
                                   laser_cloud_flat->points[closestPointInd].y,
                                   laser_cloud_flat->points[closestPointInd].z);
            Vector3d lidar_point_m(laser_cloud_flat->points[minPointInd2].x,
                                   laser_cloud_flat->points[minPointInd2].y,
                                   laser_cloud_flat->points[minPointInd2].z);
            Vector3d lidar_point_n(laser_cloud_flat->points[minPointInd3].x,
                                   laser_cloud_flat->points[minPointInd3].y,
                                   laser_cloud_flat->points[minPointInd3].z);

            // Check whether the distance of lidar points is small
            if ((lidar_point_l - vision_point).norm() > DISTANCE_THRES ||
                (lidar_point_m - vision_point).norm() > DISTANCE_THRES ||
                (lidar_point_n - vision_point).norm() > DISTANCE_THRES)
            {
                fail_cnt++;
                continue;
            }

            vector<Vector3d> pts;
            pts.reserve(4);
            pts.push_back(lidar_point_l);
            pts.push_back(lidar_point_m);
            pts.push_back(lidar_point_n);
            pts.push_back(vision_point);

            int id = int(vins_cloud->points[i].intensity);
            pts_frames[frame_count]->flat_pts_l.insert(pair<int, vector<Vector3d>>(id, pts));
        }
    }

    if (fail_cnt > 0)
        ROS_WARN_STREAM("Possible outlier number:" << fail_cnt);
}
void PointManager::preprocessing(const VinsType &vins_msg, const LaserType &laser_msg)
{
    exParamInit(vins_msg);
    if (!pts_frames[frame_count] && !poses_frames[frame_count])
    {
        pts_frames[frame_count] = new PointPerFrame{vins_msg.first.first.first, vins_msg.first.first.second, laser_msg};
        poses_frames[frame_count] = new FramePose{vins_msg.first.second, laser_msg.first};
    }
#ifdef ALOAM_GUESS
    // the transformation changed from lidar frame to body frame
    PointUtility::Tf transfo_b_l_inv = PointUtility::inverse(transfo_b_l);
    poses_frames[frame_count]->Pose = PointUtility::addTransfo(poses_frames[frame_count]->Pose, transfo_b_l_inv);
    poses_frames[frame_count]->Pose = PointUtility::addTransfo(transfo_b_l, poses_frames[frame_count]->Pose);
#endif //ALOAM_GUESS

    // Transform vins point cloud to lidar frame at cj moment
    pts_frames[frame_count]->f_pts_bj = PointUtility::transfoPtMap2End(poses_frames[frame_count]->Pose, pts_frames[frame_count]->f_pts_w);
    pts_frames[frame_count]->f_pts_lj = PointUtility::transfoPtMap2End(transfo_b_l, pts_frames[frame_count]->f_pts_bj);

    // Filter vision cloud outside the range of lidar cloud
    PointUtility::filterVerticalRange(pts_frames[frame_count]->f_pts_lj, UPWARD_ANGLE, DOWNWARD_ANGLE); // for depth association
    PointUtility::filterYawRange(pts_frames[frame_count]->f_pts_lj, CW_ANGLE, CCW_ANGLE);

    if (ALIGN_CAMERA_TIME)
        alignTransfo(transfo_cj_c_near); // project lidar cloud to lidar frame at cj time

    // Use pcl type to conduct kdtree search
    vins_cloud_filtered->clear();
    PointUtility::map2Cloud(pts_frames[frame_count]->f_pts_lj, vins_cloud_filtered);
    depthAssociation(pts_frames[frame_count]->laser_cloud_flat, vins_cloud_filtered);
}

void PointManager::guessTransform()
{
#ifdef ALOAM_GUESS
    PointUtility::Tf transfo_b_l_inv = PointUtility::inverse(transfo_b_l);
    PointUtility::Tf transfo_l0_bi = PointUtility::addTransfo(poses_frames[frame_count - 1]->pose_aloam, transfo_b_l_inv);
    PointUtility::Tf transfo_l0_bj = PointUtility::addTransfo(poses_frames[frame_count]->pose_aloam, transfo_b_l_inv);
    PointUtility::Tf transfo_bi_bj = PointUtility::findF2FTransfo(transfo_l0_bi, transfo_l0_bj);
#else
    PointUtility::Tf transfo_bi_bj = PointUtility::findF2FTransfo(poses_frames[frame_count - 1]->pose_vins,
                                                                  poses_frames[frame_count]->pose_vins);
#endif //ALOAM_GUESS
    poses_frames[frame_count]->Pose = PointUtility::addTransfo(poses_frames[frame_count - 1]->Pose, transfo_bi_bj);
}

void PointManager::prepareLocalmap()
{
    PointType pose_init;
    ROS_DEBUG_STREAM("frame count:" << frame_count);
    for (int i = frame_count; (i > (frame_count - 2)) && (i > -1); --i)
    {
        ROS_DEBUG_STREAM("id:" << i);
        pose_init.x = poses_frames[i]->Pose.t.x();
        pose_init.y = poses_frames[i]->Pose.t.y();
        pose_init.z = poses_frames[i]->Pose.t.z();

        ROS_DEBUG_STREAM("Keyframe vector size is " << keyframe_position->points.size());

        vector<int> history_keyframe_ids;
        vector<float> keyframe_sq_distance;
        keyframes_search->nearestKSearch(pose_init, 3, history_keyframe_ids, keyframe_sq_distance);
        for (int j = 0; j < int(history_keyframe_ids.size()); j++)
        {
            int id = history_keyframe_ids[j];
            *pts_frames[i]->local_map_corner += *laserCloudCorner_vec[id];
            *pts_frames[i]->local_map_flat += *laserCloudSurface_vec[id];
        }

        downsampling(pts_frames[i]->local_map_corner, pts_frames[i]->local_map_corner, lineRes);
        downsampling(pts_frames[i]->local_map_flat, pts_frames[i]->local_map_flat, planeRes);
    }
    ROS_DEBUG("Finish to preparation");
}

PtMap PointManager::guessFeature()
{
    PtMap predict_f_pts_cj;
    // Initialize the lidar f2f transformation
    if (ESTIMATE_LC_TD && !ALIGN_CAMERA_TIME)
    {
        PointUtility::Tf transfo_li_lj = PointUtility::findF2FTransfo(poses_frames[(frame_count - 1)]->pose_aloam, poses_frames[frame_count]->pose_aloam);

        // Want to initialize the depth of features in the image_j
        pts_frames[frame_count]->tmp_f_pts_lj = PointUtility::transfoPtMap2End(transfo_li_lj, pts_frames[(frame_count - 1)]->f_pts_lj);
        predict_f_pts_cj = PointUtility::transfoPtMap2End(transfo_l_c[frame_count], pts_frames[frame_count]->tmp_f_pts_lj);
    }
    else
    {
        // In Bundle Adjustment, the following code needs to be modified
        PointUtility::Tf transfo_bi_bj = PointUtility::findF2FTransfo(poses_frames[(frame_count - 1)]->pose_vins, poses_frames[frame_count]->pose_vins);
        PtMap tmp_f_pts_bj = PointUtility::transfoPtMap2End(transfo_bi_bj, pts_frames[(frame_count - 1)]->f_pts_bj);
        predict_f_pts_cj = PointUtility::transfoPtMap2End(transfo_b_c[frame_count], tmp_f_pts_bj);

        // Output result to rviz
        pts_frames[frame_count]->tmp_f_pts_lj = PointUtility::transfoPtMap2End(transfo_b_l, tmp_f_pts_bj);
    }
    return predict_f_pts_cj;
}

void PointManager::downsampling(PtCloud cloud_in, PtCloud cloud_out, double res)
{
    pcl::VoxelGrid<PointType> vox;
    vox.setInputCloud(cloud_in);
    vox.setLeafSize(res, res, res);
    vox.filter(*cloud_out);
}

vector<Vector3d> PointManager::getF2FflatCorres(const PointType point)
{
    vector<Vector3d> pts_found;
    vector<int> pt_ids;
    vector<float> pt_search_sq_distance;

    f2f_search_flat->nearestKSearch(point, 1, pt_ids, pt_search_sq_distance);

    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
    if (pt_search_sq_distance[0] < DISTANCE_SQ_THRESHOLD)
    {
        closestPointInd = pt_ids[0];

        // get closest point's scan ID
        int closestPointScanID = int(pts_frames[frame_count - 1]->laser_cloud_flat->points[closestPointInd].intensity);
        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

        // search in the direction of increasing scan line
        for (int j = closestPointInd + 1; j < (int)pts_frames[frame_count - 1]->laser_cloud_flat->points.size(); ++j)
        {
            // if not in nearby scans, end the loop
            if (int(pts_frames[frame_count - 1]->laser_cloud_flat->points[j].intensity) >
                (closestPointScanID + NEARBY_SCAN))
                break;

            double pointSqDis = pow((pts_frames[frame_count - 1]->laser_cloud_flat->points[j].x - point.x), 2) +
                                pow((pts_frames[frame_count - 1]->laser_cloud_flat->points[j].y - point.y), 2) +
                                pow((pts_frames[frame_count - 1]->laser_cloud_flat->points[j].z - point.z), 2);

            // if in the same or lower scan line
            if (int(pts_frames[frame_count - 1]->laser_cloud_flat->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
            {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
            }
            // if in the higher scan line
            else if (int(pts_frames[frame_count - 1]->laser_cloud_flat->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
            {
                minPointSqDis3 = pointSqDis;
                minPointInd3 = j;
            }
        }

        // search in the direction of decreasing scan line
        for (int j = closestPointInd - 1; j >= 0; --j)
        {
            // if not in nearby scans, end the loop
            if (int(pts_frames[frame_count - 1]->laser_cloud_flat->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                break;

            double pointSqDis = pow((pts_frames[frame_count - 1]->laser_cloud_flat->points[j].x - point.x), 2) +
                                pow((pts_frames[frame_count - 1]->laser_cloud_flat->points[j].y - point.y), 2) +
                                pow((pts_frames[frame_count - 1]->laser_cloud_flat->points[j].z - point.z), 2);

            // if in the same or higher scan line
            if (int(pts_frames[frame_count - 1]->laser_cloud_flat->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
            {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
            }
            else if (int(pts_frames[frame_count - 1]->laser_cloud_flat->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
            {
                // find nearer point
                minPointSqDis3 = pointSqDis;
                minPointInd3 = j;
            }
        }

        if (minPointInd2 >= 0 && minPointInd3 >= 0)
        {
            Vector3d last_point_a(pts_frames[frame_count - 1]->laser_cloud_flat->points[closestPointInd].x,
                                  pts_frames[frame_count - 1]->laser_cloud_flat->points[closestPointInd].y,
                                  pts_frames[frame_count - 1]->laser_cloud_flat->points[closestPointInd].z);
            Vector3d last_point_b(pts_frames[frame_count - 1]->laser_cloud_flat->points[minPointInd2].x,
                                  pts_frames[frame_count - 1]->laser_cloud_flat->points[minPointInd2].y,
                                  pts_frames[frame_count - 1]->laser_cloud_flat->points[minPointInd2].z);
            Vector3d last_point_c(pts_frames[frame_count - 1]->laser_cloud_flat->points[minPointInd3].x,
                                  pts_frames[frame_count - 1]->laser_cloud_flat->points[minPointInd3].y,
                                  pts_frames[frame_count - 1]->laser_cloud_flat->points[minPointInd3].z);

            pts_found.push_back(last_point_a);
            pts_found.push_back(last_point_b);
            pts_found.push_back(last_point_c);
        }
        else
            return pts_found;
    }
    return pts_found;
}

vector<Vector3d> PointManager::getF2McornerCorres(const PointType point, int frame_id)
{
    vector<Vector3d> pts_found;
    vector<int> pt_ids;
    vector<float> pt_search_sq_distance;

    f2m_search_corner->nearestKSearch(point, 5, pt_ids, pt_search_sq_distance);
    if (pt_search_sq_distance[4] < 1.0)
    {
        vector<Vector3d> near_corners;
        Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++)
        {
            Vector3d tmp(pts_frames[frame_id]->local_map_corner->points[pt_ids[j]].x,
                         pts_frames[frame_id]->local_map_corner->points[pt_ids[j]].y,
                         pts_frames[frame_id]->local_map_corner->points[pt_ids[j]].z);
            center = center + tmp;
            near_corners.push_back(tmp);
        }
        center = center / 5.0;

        Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++)
        {
            Eigen::Matrix<double, 3, 1> zero_mean = near_corners[j] - center;
            cov_mat = cov_mat + zero_mean * zero_mean.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
        Eigen::Vector3d cur_pt_world(point.x, point.y, point.z);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
        {
            Eigen::Vector3d point_on_line = center;
            Eigen::Vector3d point_a, point_b;
            point_a = 0.1 * unit_direction + point_on_line;
            point_b = -0.1 * unit_direction + point_on_line;

            // if (((point_a - cur_pt_world).norm()) < ((point_b - cur_pt_world).norm()))
            {
                pts_found.push_back(point_a);
                pts_found.push_back(point_b);
            }
            // else
            // {
            //     pts_found.push_back(point_b);
            //     pts_found.push_back(point_a);
            // }
        }
    }
    return pts_found;
}

VectorXd PointManager::getF2MflatCorres(const PointType point, int frame_id)
{
    VectorXd norm_valid;
    vector<int> pt_ids;
    vector<float> pt_search_sq_distance;
    f2m_search_flat->nearestKSearch(point, 5, pt_ids, pt_search_sq_distance);
    Matrix<double, 5, 3> mat_A0;
    Matrix<double, 5, 1> mat_B0 = -1 * Matrix<double, 5, 1>::Ones();
    if (pt_search_sq_distance[4] < 1.0)
    {

        for (int j = 0; j < 5; j++)
        {
            mat_A0(j, 0) = pts_frames[frame_id]->local_map_flat->points[pt_ids[j]].x;
            mat_A0(j, 1) = pts_frames[frame_id]->local_map_flat->points[pt_ids[j]].y;
            mat_A0(j, 2) = pts_frames[frame_id]->local_map_flat->points[pt_ids[j]].z;
            //printf(" pts %f %f %f ", mat_A0(j, 0), mat_A0(j, 1), mat_A0(j, 2));
        }
        // find the norm of plane
        Vector3d norm = mat_A0.colPivHouseholderQr().solve(mat_B0);
        double negative_OA_dot_norm = 1 / norm.norm();
        norm.normalize();

        // Here n(pa, pb, pc) is unit norm of plane
        bool plane_valid = true;
        for (int j = 0; j < 5; j++)
        {
            // if OX * n > 0.2, then plane is not fit well
            if (fabs(norm(0) * pts_frames[frame_id]->local_map_flat->points[pt_ids[j]].x +
                     norm(1) * pts_frames[frame_id]->local_map_flat->points[pt_ids[j]].y +
                     norm(2) * pts_frames[frame_id]->local_map_flat->points[pt_ids[j]].z + negative_OA_dot_norm) > 0.2)
            {
                // double dis = fabs(norm(0) * pts_frames[frame_id]->local_map_flat->points[pointSearchInd[j]].x +
                //                   norm(1) * pts_frames[frame_id]->local_map_flat->points[pointSearchInd[j]].y +
                //                   norm(2) * pts_frames[frame_id]->local_map_flat->points[pointSearchInd[j]].z + negative_OA_dot_norm);
                // ROS_DEBUG_STREAM("fail diast:" << dis);
                plane_valid = false;
                break;
            }
        }
        if (plane_valid)
        {
            norm_valid.resize(4);
            norm_valid.head<3>() = norm;
            norm_valid(3) = negative_OA_dot_norm;
            // ROS_DEBUG_STREAM("vector size:" << norm_valid.size());
        }
    }

    return norm_valid;
}

void PointManager::shiftPtsWindow()
{
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        transfo_l_c[i] = transfo_l_c[i + 1];
        transfo_b_c[i] = transfo_b_c[i + 1];

        std::swap(pts_frames[i], pts_frames[i + 1]);
        std::swap(poses_frames[i], poses_frames[i + 1]);
    }
    transfo_l_c[WINDOW_SIZE] = transfo_l_c[WINDOW_SIZE - 1];
    transfo_b_c[WINDOW_SIZE] = transfo_b_c[WINDOW_SIZE - 1];

    pts_frames[WINDOW_SIZE]->laser_cloud_corner->clear();
    pts_frames[WINDOW_SIZE]->laser_cloud_flat->clear();
    pts_frames[WINDOW_SIZE]->laser_cloud_full->clear();
    pts_frames[WINDOW_SIZE]->local_map_corner->clear();
    pts_frames[WINDOW_SIZE]->local_map_flat->clear();

    delete pts_frames[WINDOW_SIZE];
    delete poses_frames[WINDOW_SIZE];

    pts_frames[WINDOW_SIZE] = NULL;
    poses_frames[WINDOW_SIZE] = NULL;
}

int PointManager::findUpdateFrameId(const PointType pose_current)
{
    int update_id = -1;
    // Need to input the cloud for kdtree search
    if (laserCloudCorner_vec.empty())
        return update_id;

    vector<int> history_keyframe_ids;
    vector<float> keyframe_sq_distance;
    // ROS_DEBUG("m2.1");
    keyframes_search->radiusSearch(pose_current, MIN_RADIUS, history_keyframe_ids, keyframe_sq_distance, 0);

    // If the distance between the current frame and the history frame is small
    if (!history_keyframe_ids.empty())
    {
        // Keyframe update after a long period
        int closed_id = history_keyframe_ids[0];
        if ((poses_frames[frame_count]->time - keyframe_time_poses->points[closed_id].time) > MIN_TIME)
            return closed_id;
        else
        {
            ROS_DEBUG("Not update the keyframe in short period");
            update_id = -2;
        }
        // Don't update the keyframe when the difference of height
        if (abs(pose_current.z - keyframe_position->points[closed_id].z) > MIN_HEIGHT)
        {
            ROS_DEBUG("Update the keyframe");
            update_id = -1;
        }
    }

    return update_id;
}

void PointManager::eraseCloud(int id)
{
    pcl::PointCloud<PointType>::iterator index_kp = keyframe_position->points.begin();
    keyframe_position->erase(index_kp + id);

    pcl::PointCloud<PointXYZIT>::iterator index_ktp = keyframe_time_poses->points.begin();
    keyframe_time_poses->erase(index_ktp + id);

    vector<PtCloud>::iterator index_c = laserCloudCorner_vec.begin();
    laserCloudCorner_vec.erase(index_c + id);

    vector<PtCloud>::iterator index_s = laserCloudSurface_vec.begin();
    laserCloudSurface_vec.erase(index_s + id);
}

void PointManager::updateGlobalMap()
{
    // Initialize the variables
    PointType pose_current;
    pose_current.x = poses_frames[frame_count]->Pose.t.x();
    pose_current.y = poses_frames[frame_count]->Pose.t.y();
    pose_current.z = poses_frames[frame_count]->Pose.t.z();

    PointXYZIT point_t_3d;
    point_t_3d.x = pose_current.x;
    point_t_3d.y = pose_current.y;
    point_t_3d.z = pose_current.z;
    point_t_3d.time = poses_frames[frame_count]->time;

    PointUtility::Tf transfo_w_lj = PointUtility::addTransfo(poses_frames[frame_count]->Pose, transfo_b_l);

    PtCloud tmp_cloud_c(new pcl::PointCloud<PointType>());
    PtCloud tmp_cloud_s(new pcl::PointCloud<PointType>());

    int tmp_pts_num = pts_frames[frame_count]->laser_cloud_corner->points.size();
    tmp_cloud_c->points.resize(tmp_pts_num);
    tmp_cloud_c->width = tmp_pts_num;
    tmp_cloud_c->height = 1;

    tmp_pts_num = pts_frames[frame_count]->laser_cloud_flat->points.size();
    tmp_cloud_s->points.resize(tmp_pts_num);
    tmp_cloud_s->width = tmp_pts_num;
    tmp_cloud_s->height = 1;

    // Maintain size
    if (laserCloudCorner_vec.size() == 4851)
    {
        ROS_FATAL("Beyond the limit of map size!!");
        eraseCloud(0);
    }

    // Detect whelther there are the history frame which is near with the current frame
    int update_id = findUpdateFrameId(pose_current);
    if (update_id == -2)
    {
        tmp_cloud_c->clear();
        tmp_cloud_s->clear();
        return;
    }
    else if (update_id == -1)
    {
        ROS_DEBUG("Add the new keyframe");
        PointUtility::transfoCloud(transfo_w_lj,
                                   pts_frames[frame_count]->laser_cloud_corner,
                                   tmp_cloud_c);
        PointUtility::transfoCloud(transfo_w_lj,
                                   pts_frames[frame_count]->laser_cloud_flat,
                                   tmp_cloud_s);

        keyframe_position->points.push_back(pose_current);
        keyframe_time_poses->points.push_back(point_t_3d);
        laserCloudCorner_vec.push_back(tmp_cloud_c);
        laserCloudSurface_vec.push_back(tmp_cloud_s);
        // ROS_DEBUG("m3");
    }
    else
    {
        ROS_DEBUG("Merge the keyframe cloud");
        PointUtility::transfoCloud(transfo_w_lj,
                                   pts_frames[frame_count]->laser_cloud_corner,
                                   tmp_cloud_c);
        PointUtility::transfoCloud(transfo_w_lj, pts_frames[frame_count]->laser_cloud_flat, tmp_cloud_s);

        *tmp_cloud_c += *laserCloudCorner_vec[update_id];
        *tmp_cloud_s += *laserCloudSurface_vec[update_id];

        downsampling(tmp_cloud_c, tmp_cloud_c, lineRes);
        downsampling(tmp_cloud_s, tmp_cloud_s, planeRes);

        keyframe_position->points.push_back(pose_current);
        keyframe_time_poses->points.push_back(point_t_3d);
        laserCloudCorner_vec.push_back(tmp_cloud_c);
        laserCloudSurface_vec.push_back(tmp_cloud_s);

        eraseCloud(update_id);
        // ROS_DEBUG("m5");
    }

    // Initialize the kdtree for searching the history keyframe
    keyframes_search->setInputCloud(keyframe_position);
    // ROS_DEBUG("m6");
}