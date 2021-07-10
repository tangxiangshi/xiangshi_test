#include "vision_TEST/lvio_estimator_node.h"

LvioEstimator lvio_estimator;
#ifdef SWITCH_FUNCTION
LioEstimator lio_estimator;
#endif
std::mutex m_LVI_buf;
std::mutex m_LVI_estimator;
std::mutex m_state;

std_msgs::Header lidar_header; // to update state

int sum_of_wait_lvio = 0;
int sum_of_wait_init = 0;
int drop_cnt = 0; // count for how many datas we throwed
int lidar_id = -1;

double current_time = -1; // current time : update in pre-process of processImu
double last_imu_t = 0;

queue<VinsType> vins_buf;
queue<sensor_msgs::ImuConstPtr> imu_raw_buf;
queue<LaserType> loam_buf;

void setPose(const lvio_ros_msgs::PoseTwist pose_speed, Eigen::Quaterniond &q, Eigen::Vector3d &t)
{
    q.w() = pose_speed.orientation.w;
    q.x() = pose_speed.orientation.x;
    q.y() = pose_speed.orientation.y;
    q.z() = pose_speed.orientation.z;

    t.x() = pose_speed.position.x;
    t.y() = pose_speed.position.y;
    t.z() = pose_speed.position.z;
}

void inputImage(const lvio_ros_msgs::PointCloud3 img, map<int, Eigen::Matrix<double, 7, 1>> &image)
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
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        image.insert(pair<int, Eigen::Matrix<double, 7, 1>>(feature_id, xyz_uv_velocity));
    }
}

void inputVinsPtCloud(const sensor_msgs::PointCloudConstPtr vision_cloud, map<int, Eigen::Vector3d> &vision_points)
{
    for (unsigned i = 0; i < vision_cloud->points.size(); i++)
    {
        double feature_id = vision_cloud->channels[0].values[i];
        Eigen::Vector3d point;
        point << vision_cloud->points[i].x, vision_cloud->points[i].y, vision_cloud->points[i].z;
        vision_points.insert(pair<int, Eigen::Vector3d>(feature_id, point));
    }
}

void loam_handler(const sensor_msgs::PointCloud2ConstPtr &edge, const sensor_msgs::PointCloud2ConstPtr &flat, const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::PointCloud2ConstPtr &laser_points)
{
    m_LVI_buf.lock();
    FeatureType laser_feature_all(edge, flat);
    LaserType laser_data_all(std::make_pair(odom, std::make_pair(laser_points, laser_feature_all)));
    loam_buf.push(laser_data_all);
    m_LVI_buf.unlock();
}

// For initialization
void vins_handler(const lvio_ros_msgs::CorrectDataConstPtr &state, const sensor_msgs::PointCloudConstPtr &img, const sensor_msgs::PointCloudConstPtr &cloud, const lvio_ros_msgs::TdConstPtr &td, const nav_msgs::OdometryConstPtr &ric)
{
    m_LVI_buf.lock();
    // Store correct pose and velocity
    lvio_ros_msgs::PoseTwist tmp_pose_vel;
    tmp_pose_vel.header = state->header;
    tmp_pose_vel.position = state->position;
    tmp_pose_vel.orientation = state->orientation;
    tmp_pose_vel.velocity = state->velocity;

    // Store img with td
    lvio_ros_msgs::PointCloud3 img_td;
    img_td.header = img->header;
    img_td.cur_td = td->cur_td;
    img_td.points = img->points;
    img_td.channels = img->channels;
    VisionType features(img_td, cloud);
    VinsType vins_data(pair<VisionType, lvio_ros_msgs::PoseTwist>(features, tmp_pose_vel), ric);
    vins_buf.push(vins_data);

    m_LVI_buf.unlock();
}

void imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
    if (msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_FATAL("imu messages in disorder!");
        return;
    }
    last_imu_t = msg->header.stamp.toSec();
    m_LVI_buf.lock();
    imu_raw_buf.push(msg);
    m_LVI_buf.unlock();

    /*if (lvio_estimator.solver_flag == NON_LINEAR)
    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(msg);
        pubLatestOdometry(tmp_P, tmp_Q, tmp_V, msg);
    }*/
}

LvioData getInitMeasurements()
{
    LvioData measurements;
    while (true)
    {
#ifdef DEBUG
        if (imu_buf.empty())
        {
            ROS_WARN("Preprocess doesn't correct imu data!");
            return measurements;
        }
        if (loam_buf.empty())
        {
            ROS_WARN("Not receive loam data!");
            return measurements;
        }
#else
        if (imu_raw_buf.empty() || loam_buf.empty())
            return measurements;
#endif
        // Check whelter data in container is enough or not, the latest time of data should larger than lidar
        double time_edge = loam_buf.front().second.second.first->header.stamp.toSec();
        // double time_flat = loam_buf.front().second.second.second->header.stamp.toSec();
        if (!(imu_raw_buf.back()->header.stamp.toSec() > time_edge + lvio_estimator.lidar_td))
        {
            // ROS_WARN("Collect imu data error!");
            return measurements;
        }

        // Data needs to be earlier than lidar msg for interpolation
        if (!(imu_raw_buf.front()->header.stamp.toSec() < time_edge + lvio_estimator.lidar_td))
        {
            ROS_WARN("throw lidar msg, only should happen at the beginning ==for imu==");
            loam_buf.pop();
            drop_cnt++;
            continue;
        }

        // Construct imu vector
        sensor_msgs::PointCloud2ConstPtr corner_msg = loam_buf.front().second.second.first;
        sensor_msgs::PointCloud2ConstPtr flat_msg = loam_buf.front().second.second.second;
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while ((imu_raw_buf.front()->header.stamp.toSec() < corner_msg->header.stamp.toSec() + lvio_estimator.lidar_td) && (imu_raw_buf.front()->header.stamp.toSec() < flat_msg->header.stamp.toSec() + lvio_estimator.lidar_td))
        {
            IMUs.emplace_back(imu_raw_buf.front());
            imu_raw_buf.pop();
        }
        IMUs.emplace_back(imu_raw_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two lidar frame");

        // Construct Feature cloud,pose vector
        std::vector<VinsType> Vins_data_all;

        // Store data in measurement container.It will be end of getMeasurement step
        CorrectDataType correct_data_all(IMUs, Vins_data_all);
        measurements.emplace_back(std::make_pair(correct_data_all, loam_buf.front()));
        loam_buf.pop();

        // Clear data buffer if contain more than one measurements
        if (!loam_buf.empty())
            ROS_WARN("DROP LIDAR IN ESTIMATION FOR REAL TIME PERFORMANCE, DROP LIDAR COUNT : %d", drop_cnt);
    }
    return measurements;
}

/**
 * @brief:Align lidar msgs
 */
bool data_init = true;
VinsType tmp_vins_msg; // prevent "measurements" is empty
LvioData combineMeasurements()
{
    LvioData measurements;
    while (true)
    {
#ifdef SWITCH_FUNCTION
        if (vision_data.empty())
        {
            if (imu_data.empty())
            {
                ALOAM_flag = true;
                break;
            }
            else
            {
                LIO_flag = true;
                break;
            }
        }
#endif
#ifdef DEBUG
        if (imu_buf.empty())
        {
            ROS_WARN("Preprocess doesn't correct imu data!");
            return measurements;
        }
        if (vins_buf.empty())
        {
            ROS_WARN("Not receive vins data!");
            return measurements;
        }
        if (loam_buf.empty())
        {
            ROS_WARN("Not receive loam data!");
            return measurements;
        }
#else
        if (imu_raw_buf.empty() || vins_buf.empty() || loam_buf.empty())
            return measurements;
#endif
        // Check whelter data in container is enough or not, the latest time of data should larger than lidar
        double time_edge = loam_buf.front().second.second.first->header.stamp.toSec();
        double time_flat = loam_buf.front().second.second.second->header.stamp.toSec();
        if (!(imu_raw_buf.back()->header.stamp.toSec() > time_edge + lvio_estimator.lidar_td))
        {
            // ROS_WARN("Collect imu data error!");
            return measurements;
        }
        lvio_ros_msgs::PointCloud3 img_back = vins_buf.back().first.first.first;
        if (!(img_back.header.stamp.toSec() + img_back.cur_td > time_edge + lvio_estimator.lidar_td) || !(img_back.header.stamp.toSec() + img_back.cur_td > time_flat + lvio_estimator.lidar_td))
        {
            // ROS_WARN("Collect image data error!");
            return measurements;
        }

        // Data needs to be earlier than lidar msg for interpolation
        lvio_ros_msgs::PointCloud3 img_front = vins_buf.front().first.first.first;
        if (!(imu_raw_buf.front()->header.stamp.toSec() < time_edge + lvio_estimator.lidar_td))
        {
            ROS_WARN("throw lidar msg, only should happen at the beginning ==for imu==");

            // formulate f2f odometry from last to current (reference: LIO)
            // {
            //
            // }

            loam_buf.pop();
            drop_cnt++;
            continue;
        }
        if (!(img_front.header.stamp.toSec() + img_front.cur_td < time_edge + lvio_estimator.lidar_td))
        {
            if (data_init)
            {
                ROS_WARN("throw lidar msg, only should happen at the beginning ==for image==");
                data_init = false;
                loam_buf.pop();
                drop_cnt++;
                continue;
            }
            else
            {
                if (img_front.header.stamp.toSec() + img_front.cur_td - time_edge - lvio_estimator.lidar_td > 0.1)
                {
                    ROS_WARN("losing img. Need to use LIO mode!!!!!!");
                    // use lio mode
                }
            }
        }

        // Construct imu vector
        sensor_msgs::PointCloud2ConstPtr corner_msg = loam_buf.front().second.second.first;
        sensor_msgs::PointCloud2ConstPtr flat_msg = loam_buf.front().second.second.second;
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while ((imu_raw_buf.front()->header.stamp.toSec() < corner_msg->header.stamp.toSec() + lvio_estimator.lidar_td) && (imu_raw_buf.front()->header.stamp.toSec() < flat_msg->header.stamp.toSec() + lvio_estimator.lidar_td))
        {
            IMUs.emplace_back(imu_raw_buf.front());
            imu_raw_buf.pop();
        }
        IMUs.emplace_back(imu_raw_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two lidar frame");

        // Construct Feature cloud,pose vector
        std::vector<VinsType> Vins_data_all;
        while ((vins_buf.front().first.first.first.header.stamp.toSec() + vins_buf.front().first.first.first.cur_td <= corner_msg->header.stamp.toSec() + lvio_estimator.lidar_td) && (vins_buf.front().first.first.first.header.stamp.toSec() + vins_buf.front().first.first.first.cur_td <= flat_msg->header.stamp.toSec() + lvio_estimator.lidar_td))
        {
            tmp_vins_msg = vins_buf.front();
            vins_buf.pop();
        }
        Vins_data_all.emplace_back(tmp_vins_msg);
        Vins_data_all.emplace_back(vins_buf.front());
        if (!(Vins_data_all.size() > 1))
            ROS_WARN("========Design with bugs!!!!!!========");

        // Store data in measurement container.It will be end of getMeasurement step
        CorrectDataType correct_data_all(IMUs, Vins_data_all);
        measurements.emplace_back(std::make_pair(correct_data_all, loam_buf.front()));
        loam_buf.pop();

        // Clear data buffer if contain more than one measurements
        if (!loam_buf.empty())
        {
            // drop_cnt += loam_buf.size();
            ROS_WARN("DROP LIDAR IN ESTIMATION FOR REAL TIME PERFORMANCE, DROP LIDAR COUNT : %d", drop_cnt);
            // std::queue<LaserType> empty;
            // std::swap(loam_buf, empty);
        }
    }
    return measurements;
}

// thread: laser-visual-inertial odometry
void lvio_process()
{
    while (true)
    {
        LvioData measurements;
        m_LVI_buf.lock();
        if (lvio_estimator.solver_flag == LvioEstimator::SolverFlag::INITIAL)
            measurements = getInitMeasurements();
        else
            measurements = combineMeasurements();
        m_LVI_buf.unlock();

        // Check whelther measurements are enough to do lvio
        if (measurements.empty())
            continue;
        if (measurements.size() != 1)
            ROS_FATAL("MEASUREMENT SIZE IS LARGER THAN 1, IT MAY CAUSE DELAY PROBLEM");
        if (measurements.front().first.first.empty())
        {
            ROS_FATAL("IMU VECTOR IS EMPTY");
            continue;
        }

        // Check whelther need to use lio mode or VINS
        if (measurements.front().first.second.empty() && !(LvioEstimator::SolverFlag::INITIAL))
        {
            ROS_FATAL("VISION VECTOR IS EMPTY");
            // Need to use lio mode
        }
        else if (measurements.front().second.second.second.second->data.size() < PTS_NUM_THRES)
        {
            ROS_FATAL("LIDAR VECTOR IS EMPTY");
            // publish VIO data
        }

        m_LVI_estimator.lock();
        /**
          * @brief: Interpolate the imu measurement and vins pose to edge or plane cloud
          */
        for (auto &measurement : measurements)
        {
            lidar_id++;

            // First part: handle imu msg and accumulate delta p, q, v on imu_i frame
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            double edge_msg_t = measurement.second.second.second.first->header.stamp.toSec() + lvio_estimator.lidar_td;
            double flat_msg_t = measurement.second.second.second.second->header.stamp.toSec() + lvio_estimator.lidar_td;
            for (auto &imu_msg : measurement.first.first)
            {
                // Handle imu vector with processIMU
                double t = imu_msg->header.stamp.toSec();
                if (t <= edge_msg_t && t <= flat_msg_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time; // approximately 0, 1/imu_frequency
                    current_time = t;             // update current time with imu time
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    lvio_estimator.processImu(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
                else
                {
                    // Interpolation
                    double dt_1 = edge_msg_t - current_time;
                    double dt_2 = t - edge_msg_t;
                    current_time = edge_msg_t;

                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);

                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    lvio_estimator.processImu(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
            }

            if (!(lvio_estimator.solver_flag == LvioEstimator::SolverFlag::INITIAL))
            {
                // Second part: choose the img which is the nearest to lidar cloud
                map<int, Eigen::Matrix<double, 7, 1>> image;
                map<int, Eigen::Vector3d> vision_points;
                map<int, Eigen::Vector3d> v_pts_at_w;
                Eigen::Quaterniond q_w2bi;
                Eigen::Vector3d t_w2bi;
                if (measurement.first.second.size() != 2)
                    ROS_WARN("vins getmeasurement error!!!");
                else
                {
                    // Generally, img period is 0.05(sec)
                    lvio_ros_msgs::PointCloud3 first_img = measurement.first.second[0].first.first.first;
                    lvio_ros_msgs::PointCloud3 second_img = measurement.first.second[1].first.first.first;
                    sensor_msgs::PointCloudConstPtr first_vision_cloud = measurement.first.second[0].first.first.second;
                    sensor_msgs::PointCloudConstPtr second_vision_cloud = measurement.first.second[1].first.first.second;
                    lvio_ros_msgs::PoseTwist first_pose_speed = measurement.first.second[0].first.second;
                    lvio_ros_msgs::PoseTwist second_pose_speed = measurement.first.second[1].first.second;

                    double first_img_t = first_img.header.stamp.toSec() + first_img.cur_td;
                    double second_img_t = second_img.header.stamp.toSec() + second_img.cur_td;
                    double pose_before_t = first_pose_speed.header.stamp.toSec();
                    double pose_after_t = second_pose_speed.header.stamp.toSec();
                    if (second_img_t - first_img_t > 0.06)
                        ROS_WARN("losing img");
                    if ((edge_msg_t - first_img_t) < (second_img_t - edge_msg_t))
                    {
                        inputImage(first_img, image);
                        inputVinsPtCloud(first_vision_cloud, v_pts_at_w);
                        setPose(first_pose_speed, q_w2bi, t_w2bi);
#ifdef VEL_APPROXIMATION
                        // Two mode to find the pose bewteen two known pose
                        if (fabs(first_pose_speed.velocity - second_pose_speed.velocity) > VEL_TRES)
                        {
                            //use velocity to approximation
                        }
                        else
#endif
                        {
                            double dt_1 = edge_msg_t - pose_before_t;
                            double t_scale = dt_1 / (pose_after_t - pose_before_t);
                            // need to use ncrl deltaTrans
                            // slerp for quaternion interpolation
                        }
                        // pose input processLVIO
                    }
                    else
                    {
                        inputImage(second_img, image);
                        inputVinsPtCloud(second_vision_cloud, v_pts_at_w);
                        setPose(second_pose_speed, q_w2bi, t_w2bi);
                        double dt_2 = pose_after_t - edge_msg_t;
                        double t_scale = dt_2 / (pose_after_t - pose_before_t);
                    }
                }
            }

            FeaturePerSweep feature_msg;
            pcl::fromROSMsg(*measurement.second.second.second.first, *feature_msg.laserCloudCornerLast);
            pcl::fromROSMsg(*measurement.second.second.second.second, *feature_msg.laserCloudSurfLast);
            pcl::fromROSMsg(*measurement.second.second.first, *feature_msg.laserCloudFullRes);
            feature_msg.laserOdom = *measurement.second.first;

            // Set transform on IMU/LIDAR frame from laser odometry
            Eigen::Quaterniond q_(feature_msg.laserOdom.pose.pose.orientation.w,
                                  feature_msg.laserOdom.pose.pose.orientation.x,
                                  feature_msg.laserOdom.pose.pose.orientation.y,
                                  feature_msg.laserOdom.pose.pose.orientation.z);
            Eigen::Vector3d v_(feature_msg.laserOdom.pose.pose.position.x,
                               feature_msg.laserOdom.pose.pose.position.y,
                               feature_msg.laserOdom.pose.pose.position.z);
            ncrl_tf::setTrans(q_, v_, feature_msg.Tinit2odom_L);

            // trans f2f odom from lidar_odom 2 lidar -> imu_odom 2 imu
            if (!ncrl_tf::TransOdometry(EXTRINSIC, feature_msg.Tinit2odom_L,
                                        feature_msg.Tinit2odom_I))
                ROS_WARN("Trans odom from lidar frame to imu frame fail");

            lidar_header = feature_msg.laserOdom.header;
            lvio_estimator.processLVIO(feature_msg, lidar_id);
        }

#ifdef SWITCH_FUNCTION
        else if (LIO_flag)
        {
            //lvio_estimator.processLIO();
        }
        else if (ALOAM_flag)
        {
            //publish(ALOAM)
        }
#endif
        m_LVI_estimator.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lvio_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(n);
    // lvio_estimator.setParameter();

    // Subscribe the correct imu data and image feature
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());

    // Message filter from VINS
    message_filters::Subscriber<lvio_ros_msgs::CorrectData> sub_correct_data(n, "/vins_estimator/correct_data", 100);
    message_filters::Subscriber<lvio_ros_msgs::Td> sub_td(n, "/vins_estimator/td", 100);
    message_filters::Subscriber<nav_msgs::Odometry> sub_ric(n, "/vins_estimator/extrinsic", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_vision_local_cloud(n, "/vins_estimator/vision_local_cloud", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_imgs(n, "/feature_tracker/feature", 100);

    typedef sync_policies::ApproximateTime<lvio_ros_msgs::CorrectData, sensor_msgs::PointCloud, sensor_msgs::PointCloud, lvio_ros_msgs::Td, nav_msgs::Odometry> VinsPolicy;
    Synchronizer<VinsPolicy> sync_vins(VinsPolicy(1000), sub_correct_data, sub_imgs, sub_vision_local_cloud, sub_td, sub_ric);
    sync_vins.registerCallback(boost::bind(&vins_handler, _1, _2, _3, _4, _5));

    // Subscribe each Lidar point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_LaserCloudEdgeLast(n, "/laserOdometry/laser_cloud_corner_last", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_LaserCloudFlatLast(n, "/laserOdometry/laser_cloud_surf_last", 100);
    message_filters::Subscriber<nav_msgs::Odometry> sub_LaserOdometry(n, "/laserOdometry/laser_odom_to_init", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_LaserCloudFullRes(n, "/laserOdometry/velodyne_cloud_3", 100);

    // Message filter from ALOAM
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry, sensor_msgs::PointCloud2> LoamPolicy;
    Synchronizer<LoamPolicy> sync_loam(LoamPolicy(1000), sub_LaserCloudEdgeLast, sub_LaserCloudFlatLast, sub_LaserOdometry, sub_LaserCloudFullRes);
    sync_loam.registerCallback(boost::bind(&loam_handler, _1, _2, _3, _4));

    // Extrinsic from lidar to imu
    ncrl_tf::setTransFrame("IMU", "LIDAR", EXTRINSIC);
    Eigen::Quaterniond EQ_I2L = ncrl_tf::Euler2Q(ER_I2L);
    ncrl_tf::setTrans(EQ_I2L, ET_I2L, EXTRINSIC);
    lvio_estimator.til[0] = EXTRINSIC.v;
    lvio_estimator.qil[0] = EXTRINSIC.q;

    std::thread lvio_thread;
    lvio_thread = std::thread(lvio_process);

    ros::spin();
    return 0;
}
