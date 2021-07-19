#include "depth_map_visualization.h"
#include "point_utility.h"

#define TRANSFORM_TO_WORLD

ros::Publisher pub_vins_cloud_filtered, pub_vins_guess, pub_correct_cloud;
ros::Publisher pub_regist, pub_map, pub_local_map_corner, pub_local_map_flat;
ros::Publisher pub_bug_arrow, pub_vision_corres_num;
ros::Publisher pub_livins_odometry, pub_vins_odometry;
ros::Publisher pub_path, pub_vins_path;

nav_msgs::Path path, vins_path;


PtCloud vins_pts_corrected(new pcl::PointCloud<PointType>());
PtCloud vins_pts_guess(new pcl::PointCloud<PointType>());
PtCloud laser_cloud_registed(new pcl::PointCloud<PointType>());
PtCloud laser_cloud_surround(new pcl::PointCloud<PointType>());

void registerPub(ros::NodeHandle &n)
{
    pub_vins_cloud_filtered = n.advertise<sensor_msgs::PointCloud2>("vins_point_cloud", 100);
    pub_vins_guess = n.advertise<sensor_msgs::PointCloud2>("vins_guess_cloud", 100);
    pub_correct_cloud = n.advertise<sensor_msgs::PointCloud2>("correct_image_cloud", 100);

    pub_regist = n.advertise<sensor_msgs::PointCloud2>("regist_points", 100);
    pub_map = n.advertise<sensor_msgs::PointCloud2>("map", 100);
    pub_local_map_corner = n.advertise<sensor_msgs::PointCloud2>("local_map_corner", 100);
    pub_local_map_flat = n.advertise<sensor_msgs::PointCloud2>("local_map_flat", 100);

    // Debug
    pub_bug_arrow = n.advertise<visualization_msgs::MarkerArray>("bug_point_arrow", 100);
    pub_vision_corres_num = n.advertise<geometry_msgs::PointStamped>("ok_vision_corres_num", 100);

    pub_livins_odometry = n.advertise<nav_msgs::Odometry>("livins_odometry", 10);
    pub_vins_odometry = n.advertise<nav_msgs::Odometry>("vins_odometry", 50);

    pub_path = n.advertise<nav_msgs::Path>("livins_path", 10);
    pub_vins_path = n.advertise<nav_msgs::Path>("vins_path", 20);
}

void pubCorrectCloud(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    PointUtility::map2Cloud(estimator.solved_pts, vins_pts_corrected);

#ifdef TRANSFORM_TO_WORLD
    PointUtility::Tf transfo_w_lj = PointUtility::addTransfo(estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose,
                                                             estimator.pts_manager.transfo_b_l);
    PointUtility::transfoCloud(transfo_w_lj, vins_pts_corrected, vins_pts_corrected);
#endif //TRANSFORM_TO_WORLD

    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*vins_pts_corrected, cloud_output);
    cloud_output.header.stamp = header.stamp;
    cloud_output.header.frame_id = "world";
    pub_correct_cloud.publish(cloud_output);

    vins_pts_corrected->clear();
}

void pubVinsCloudFiltered(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*estimator.pts_manager.vins_cloud_filtered, cloud_output);
    cloud_output.header.stamp = header.stamp;
    cloud_output.header.frame_id = "world";
    pub_vins_cloud_filtered.publish(cloud_output);
}

void pubGuessCloud(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    PointUtility::map2Cloud(estimator.pts_manager.pts_frames[estimator.frame_count - 1]->tmp_f_pts_lj, vins_pts_guess);

    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*vins_pts_guess, cloud_output);
    cloud_output.header.stamp = header.stamp;
    cloud_output.header.frame_id = "world";
    pub_vins_guess.publish(cloud_output);

    vins_pts_guess->clear();
}

void pubLidarCurrent(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
#ifdef TRANSFORM_TO_WORLD
    // Project the one sweep
    PointUtility::Tf transfo_w_lj = PointUtility::addTransfo(estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose,
                                                             estimator.pts_manager.transfo_b_l);
    PointUtility::transfoCloud(transfo_w_lj, estimator.pts_manager.pts_frames[estimator.frame_count - 1]->laser_cloud_full,
                               estimator.pts_manager.pts_frames[estimator.frame_count - 1]->laser_cloud_full);
#endif //TRANSFORM_TO_WORLD
    laser_cloud_registed->clear();
    laser_cloud_registed = estimator.pts_manager.pts_frames[estimator.frame_count - 1]->laser_cloud_full;

    // Publish current registered cloud
    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*laser_cloud_registed, cloud_output);
    cloud_output.header.stamp = header.stamp;
    cloud_output.header.frame_id = "world";
    pub_regist.publish(cloud_output);

    sensor_msgs::PointCloud2 local_cloud_corner;
    pcl::toROSMsg(*estimator.pts_manager.pts_frames[estimator.frame_count - 1]->local_map_corner,
                  local_cloud_corner);
    local_cloud_corner.header.stamp = header.stamp;
    local_cloud_corner.header.frame_id = "world";
    pub_local_map_corner.publish(local_cloud_corner);

    sensor_msgs::PointCloud2 local_cloud_flat;
    pcl::toROSMsg(*estimator.pts_manager.pts_frames[estimator.frame_count - 1]->local_map_flat,
                  local_cloud_flat);
    local_cloud_flat.header.stamp = header.stamp;
    local_cloud_flat.header.frame_id = "world";
    pub_local_map_flat.publish(local_cloud_flat);
}

void pubSurroundingCloud(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    // Publish map at 1/12 Hz
    // if ((estimator.pts_manager.sweep_id % WINDOW_SIZE) == 0)
    {
        ROS_WARN("re-input the map!!");
        laser_cloud_surround->clear();
        for (int i = 0; i < int(estimator.pts_manager.laserCloudCorner_vec.size()); i++)
        {
            *laser_cloud_surround += *estimator.pts_manager.laserCloudCorner_vec[i];
            *laser_cloud_surround += *estimator.pts_manager.laserCloudSurface_vec[i];
        }
    }

    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*laser_cloud_surround, cloud_output);
    cloud_output.header.stamp = header.stamp;
    cloud_output.header.frame_id = "world";
    pub_map.publish(cloud_output);
}

void pubVisualization(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    visualization_msgs::MarkerArray arrow_array;
    for (unsigned int i = 0; i < estimator.f_arrow_l.size(); i++)
    {
        geometry_msgs::Point start_point;
        start_point.x = estimator.f_arrow_l[i][0].x();
        start_point.y = estimator.f_arrow_l[i][0].y();
        start_point.z = estimator.f_arrow_l[i][0].z();

        for (int j = 1; j < 4; j++)
        {
            visualization_msgs::Marker arrow;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.header.frame_id = "world";
            arrow.header.stamp = header.stamp;
            arrow.ns = "arrow";
            arrow.action = visualization_msgs::Marker::ADD;
            arrow.pose.orientation.w = 1.0;
            arrow.id = i * 3 + j;
            arrow.scale.x = 0.01;
            arrow.scale.y = 0.01;
            arrow.scale.z = 0.0;
            arrow.color.g = 1;
            arrow.color.a = 1;

            geometry_msgs::Point end_point;
            end_point.x = estimator.f_arrow_l[i][j].x();
            end_point.y = estimator.f_arrow_l[i][j].y();
            end_point.z = estimator.f_arrow_l[i][j].z();

            arrow.points.push_back(start_point);
            arrow.points.push_back(end_point);

            arrow_array.markers.push_back(arrow);
            arrow.points.clear();
        }
        pub_bug_arrow.publish(arrow_array);
    }
}

void pubLIVOdometry(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    Eigen::Quaterniond robot_Q = estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose.q;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose.t.x();
    odometry.pose.pose.position.y = estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose.t.y();
    odometry.pose.pose.position.z = estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose.t.z();
    odometry.pose.pose.orientation.x = robot_Q.x();
    odometry.pose.pose.orientation.y = robot_Q.y();
    odometry.pose.pose.orientation.z = robot_Q.z();
    odometry.pose.pose.orientation.w = robot_Q.w();
    odometry.twist.twist.linear.x = estimator.Vs[estimator.frame_count - 1].x();
    odometry.twist.twist.linear.y = estimator.Vs[estimator.frame_count - 1].y();
    odometry.twist.twist.linear.z = estimator.Vs[estimator.frame_count - 1].z();
    pub_livins_odometry.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;

    path.header = header;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
}

void pubVINSPose(const lvio_ros_msgs::CorrectDataConstPtr &state)
{
    nav_msgs::Odometry odometry;
    odometry.header = state->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position = state->position;
    odometry.pose.pose.orientation = state->orientation;
    odometry.twist.twist.linear.x = 0; // without estimating the velocity of the robot
    odometry.twist.twist.linear.y = 0;
    odometry.twist.twist.linear.z = 0;
    pub_vins_odometry.publish(odometry);

    geometry_msgs::PoseStamped vins_pose_stamped;
    vins_pose_stamped.header = state->header;
    vins_pose_stamped.header.frame_id = "world";
    vins_pose_stamped.pose = odometry.pose.pose;

    vins_path.header = state->header;
    vins_path.header.frame_id = "world";
    vins_path.poses.push_back(vins_pose_stamped);
    pub_vins_path.publish(vins_path);
}

void pubTf(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose.t;
    correct_q = estimator.pts_manager.poses_frames[estimator.frame_count - 1]->Pose.q;

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));
}

void pubReceivedVisionNum(DepthMapEstimator &estimator, const std_msgs::Header &header)
{
    geometry_msgs::PointStamped vision_corres_num;
    vision_corres_num.header = header;
    vision_corres_num.point.x = estimator.received_num;
    vision_corres_num.point.y = 0;
    vision_corres_num.point.z = 0;
    pub_vision_corres_num.publish(vision_corres_num);
}
