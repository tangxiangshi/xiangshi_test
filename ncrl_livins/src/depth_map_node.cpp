#include <ncrl_livins/depth_map_node.h>

#define VISUAL_ALIGN
// #define LASER_ALIGN

//#define COMMAND_MODE
//#define NEAR_TIME

DepthMapEstimator estimator;
std::mutex m_buf, m_estimator, m_state;

#ifdef COMMAND_MODE
bool start_process = 0, wait2start = 1;
#endif //COMMAND_MODE
bool vins_init = 0, data_init = 1, img_init = 1;

int drop_cnt, use_index = 0;
double last_imu_t = 0, current_time = -1;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<VinsType> vins_buf;
queue<LaserType> loam_buf;
// tmp_msg
VinsType tmp_vins_msg; // prevent "measurements" is empty

void predict()
{
    //
}

void update()
{
    ROS_DEBUG("update optimization value");
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (vins_init)
    {
        if (imu_msg->header.stamp.toSec() <= last_imu_t)
        {
            ROS_WARN("imu message in disorder!");
            return;
        }
        last_imu_t = imu_msg->header.stamp.toSec();

        m_buf.lock();
        imu_buf.push(imu_msg);
        m_buf.unlock();
        // con.notify_one();

        last_imu_t = imu_msg->header.stamp.toSec();

        {
            // std::lock_guard<std::mutex> lg(m_state);
            // predict(imu_msg);
            // std_msgs::Header header = imu_msg->header;
            // header.frame_id = "world";
            // if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            //     pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
        }
    }
}

void loam_handler(const sensor_msgs::PointCloud2ConstPtr &edge,
                  const sensor_msgs::PointCloud2ConstPtr &flat,
                  const nav_msgs::OdometryConstPtr &odom,
                  const sensor_msgs::PointCloud2ConstPtr &laser_points)
{
#ifdef COMMAND_MODE
    if (start_process)
#else
    if (vins_init)
#endif //COMMAND_MODE
    {
        m_buf.lock();
        FeatureType laser_feature_all(edge, flat);
        LaserType laser_data_all(std::make_pair(odom, std::make_pair(laser_points, laser_feature_all)));
        loam_buf.push(laser_data_all);
        m_buf.unlock();
    }
}

void vins_handler(const lvio_ros_msgs::CorrectDataConstPtr &state,
                  const sensor_msgs::PointCloudConstPtr &img,
                  const sensor_msgs::PointCloudConstPtr &cloud,
                  const lvio_ros_msgs::TdConstPtr &td,
                  const nav_msgs::OdometryConstPtr &ric,
                  const geometry_msgs::PointStampedConstPtr &g)
{
#ifdef COMMAND_MODE
    if (start_process)
#endif //COMMAND_MODE
    {
        m_buf.lock();

        // Store img with td
        lvio_ros_msgs::PointCloud3 img_td;
        img_td.header = img->header;
        img_td.cur_td = td->cur_td;
        img_td.points = img->points;
        img_td.channels = img->channels;

        VisionType features(img_td, cloud);
        VinsType vins_data(pair<VisionType,
                                lvio_ros_msgs::CorrectDataConstPtr>(features, state),
                           pair<nav_msgs::OdometryConstPtr,
                                geometry_msgs::PointStampedConstPtr>(ric, g));
        vins_buf.push(vins_data);
        vins_init = 1;
        m_buf.unlock();
        pubVINSPose(state);
    }
}

LvioType getMeasurements()
{
    LvioType measurements;
    while (true)
    {
        if (imu_buf.empty() || vins_buf.empty() || loam_buf.empty())
            return measurements;
        // if (vins_buf.empty())
        // {
        //     ROS_DEBUG("vins empty");
        //     return measurements;
        // }
        // if (loam_buf.empty())
        // {
        //     ROS_DEBUG("loam empty");
        //     return measurements;
        // }
        // if (imu_buf.empty())
        // {
        //     ROS_DEBUG("imu empty");
        //     return measurements;
        // }

        // Check whelter data in container is enough or not, the latest time of data should larger than lidar
        double time_edge = loam_buf.front().second.second.first->header.stamp.toSec();
        double time_flat = loam_buf.front().second.second.second->header.stamp.toSec();
        if (!(imu_buf.back()->header.stamp.toSec() > time_edge + LIDAR_TD))
        {
            // ROS_WARN("Collect imu data error!");
            return measurements;
        }

        lvio_ros_msgs::PointCloud3 img_back = vins_buf.back().first.first.first;
        if (!(img_back.header.stamp.toSec() + img_back.cur_td > time_edge + LIDAR_TD) ||
            !(img_back.header.stamp.toSec() + img_back.cur_td > time_flat + LIDAR_TD))
        {
            // ROS_WARN("Collect image data error!");
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < time_edge + LIDAR_TD))
        {
            ROS_WARN("throw lidar msg, only should happen at the beginning ==for imu==");

            // formulate f2f odometry from last to current (reference: LIO)

            loam_buf.pop();
            drop_cnt++;
            continue;
        }

        // Data needs to be earlier than lidar msg for interpolation
        lvio_ros_msgs::PointCloud3 img_front = vins_buf.front().first.first.first;
        if (!(img_front.header.stamp.toSec() + img_front.cur_td < time_edge + LIDAR_TD))
        {
            if (data_init)
            {
                ROS_WARN("throw lidar msg, only should happen at the beginning ==for image==");
                data_init = 0;
                loam_buf.pop();
                drop_cnt++;
                continue;
            }
            else
            {
                if (img_front.header.stamp.toSec() + img_front.cur_td - time_edge - LIDAR_TD > 0.1)
                {
                    ROS_WARN("losing img. Need to use LIO mode!!!!!!");
                    // use lio mode
                }
            }
        }

        sensor_msgs::PointCloud2ConstPtr corner_msg = loam_buf.front().second.second.first;
        sensor_msgs::PointCloud2ConstPtr flat_msg = loam_buf.front().second.second.second;
#ifdef LASER_ALIGN
        // Construct imu vector
        vector<sensor_msgs::ImuConstPtr> IMUs;
        while ((imu_buf.front()->header.stamp.toSec() < corner_msg->header.stamp.toSec() + LIDAR_TD) &&
               (imu_buf.front()->header.stamp.toSec() < flat_msg->header.stamp.toSec() + LIDAR_TD))
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two lidar frame");
#endif //LASER_ALIGN

        // Construct Feature cloud,pose vector
        vector<VinsType> vins_data_all;
        while ((vins_buf.front().first.first.first.header.stamp.toSec() + vins_buf.front().first.first.first.cur_td <= corner_msg->header.stamp.toSec() + LIDAR_TD) &&
               (vins_buf.front().first.first.first.header.stamp.toSec() + vins_buf.front().first.first.first.cur_td <= flat_msg->header.stamp.toSec() + LIDAR_TD))
        {
            tmp_vins_msg = vins_buf.front();
            vins_buf.pop();
        }

        if (vins_buf.front().first.first.first.header.stamp.toSec() - tmp_vins_msg.first.first.first.header.stamp.toSec() > 0.06)
            ROS_WARN("losing img");
        vins_data_all.emplace_back(tmp_vins_msg);
        vins_data_all.emplace_back(vins_buf.front());

        if (!(vins_data_all.size() > 1))
            ROS_WARN("========Design with bugs!!!!!!========");

#ifdef VISUAL_ALIGN
        // Construct imu vector
        vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() <
               vins_buf.front().first.first.first.header.stamp.toSec() + vins_buf.front().first.first.first.cur_td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two lidar frame");
#endif //VISUAL_ALIGN

        // Store data in measurement container.It will be end of getMeasurement step
        LvPairType laser_vision_data(vins_data_all, loam_buf.front());
        measurements.emplace_back(make_pair(laser_vision_data, IMUs));
        loam_buf.pop();

        // Clear data buffer if contain more than one measurements
        if (!loam_buf.empty())
            ROS_WARN("DROP LIDAR IN ESTIMATION FOR REAL TIME PERFORMANCE, DROP LIDAR COUNT : %d", drop_cnt);
    }
    return measurements;
}

void depth_map_process()
{
    while (true)
    {
        LvioType measurements;
        m_buf.lock();
        measurements = getMeasurements();
        m_buf.unlock();

        // Check whelther measurements are enough to do lvio
        if (measurements.empty())
            continue;

        if (measurements.size() != 1)
            ROS_FATAL("MEASUREMENT SIZE IS LARGER THAN 1, IT MAY CAUSE DELAY PROBLEM");

        if (measurements.front().second.empty())
        {
            ROS_FATAL("IMU VECTOR IS EMPTY");
            continue;
        }

        // Check whelther need to use lio mode or VINS
        if (measurements.front().first.first.empty())
            ROS_FATAL("VISION VECTOR IS EMPTY");
        else if (measurements.front().first.second.second.second.second->data.size() < PTS_NUM_THRES)
            ROS_FATAL("LIDAR VECTOR IS EMPTY");

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            if (measurement.first.first.size() != 2)
                ROS_WARN("vins getmeasurement error!!!");
            else
            {
                VinsType first_vins_data = measurement.first.first[0];
                VinsType second_vins_data = measurement.first.first[1];
                double first_img_t = first_vins_data.second.second->header.stamp.toSec() + first_vins_data.first.first.first.cur_td;
                double second_img_t = second_vins_data.second.second->header.stamp.toSec() + second_vins_data.first.first.first.cur_td;
                double edge_msg_t = measurement.first.second.second.second.first->header.stamp.toSec() + LIDAR_TD;
                if (img_init)
                {
                    /**
                     * @brief: Find the input data to initialize
                     */
#ifdef NEAR_TIME
                    // Choose the consistent lead or lag image.Generally, img period is 0.05(sec)
                    if (edge_msg_t - first_img_t > second_img_t - edge_msg_t)
#endif //NEAR_TIME
                    {
                        ROS_DEBUG("second img is more near to lidar msg!!");
                        use_index = 1;
                    }
                    img_init = 0;
                }

                estimator.initSpeedBias(measurement.first.first[use_index]);
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : measurement.second)
                {
                    double t = imu_msg->header.stamp.toSec();
#ifdef LASER_ALIGN
                    if (t <= edge_msg_t)
#endif //LASER_ALIGN
#ifdef VISUAL_ALIGN
                        if (t <= second_img_t)
#endif //VISUAL_ALIGN
                        {
                            if (current_time < 0)
                                current_time = t;
                            double dt = t - current_time;
                            ROS_ASSERT(dt >= 0);
                            current_time = t;
                            dx = imu_msg->linear_acceleration.x;
                            dy = imu_msg->linear_acceleration.y;
                            dz = imu_msg->linear_acceleration.z;
                            rx = imu_msg->angular_velocity.x;
                            ry = imu_msg->angular_velocity.y;
                            rz = imu_msg->angular_velocity.z;
                            estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        }
                        else
                        {
#ifdef LASER_ALIGN
                            double dt_1 = edge_msg_t - current_time;
                            double dt_2 = t - edge_msg_t;
                            current_time = edge_msg_t;
#endif //LASER_ALIGN
#ifdef VISUAL_ALIGN
                            double dt_1 = second_img_t - current_time;
                            double dt_2 = t - second_img_t;
                            current_time = second_img_t;
#endif //VISUAL_ALIGN
                            ROS_ASSERT(dt_1 >= 0);
                            ROS_ASSERT(dt_2 >= 0);
                            ROS_ASSERT(dt_1 + dt_2 > 0);
                            double w1 = dt_2 / (dt_1 + dt_2);
                            double w2 = dt_1 / (dt_1 + dt_2);
                            dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                            dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                            dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                            rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                            ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                            rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                            estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                            //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                        }
                }
                /**
                 * @brief: Need to interpolate VINS poses to find f2f transformation
                 */
                if (ALIGN_CAMERA_TIME)
                {
                    // double delta_img_t = first_img_t - second_img_t;
                    // ROS_DEBUG_STREAM("image interval:" << delta_img_t);
                    double s = (edge_msg_t - measurement.first.first[use_index].second.first->header.stamp.toSec()) / (first_img_t - second_img_t);
                    s = abs(s);
                    estimator.setLidarTransfo(first_vins_data, second_vins_data, s, use_index);
                }
                ROS_DEBUG("Sweep id: %d", estimator.pts_manager.sweep_id);
                estimator.processLVIO(measurement.first.first[use_index], measurement.first.second);
                std_msgs::Header header;
                if (ESTIMATE_LC_TD)
                    header = measurement.first.second.first->header;
                else
                    header = estimator.headers[estimator.frame_count - 1];

                pubLidarCurrent(estimator, header);     // Publish the current point cloud
                pubSurroundingCloud(estimator, header); // Publish the global map
                pubVinsCloudFiltered(estimator, header); // We will store and use the filter cloud in window
                pubGuessCloud(estimator, header);        // Guess the tracked features in current lidar frame
                pubVisualization(estimator, header);     // Publish the depthmap association bugs
                pubCorrectCloud(estimator, header); // Check the feature position after recoved the its depth
                pubLIVOdometry(estimator, header);  // check the odomtry
                pubReceivedVisionNum(estimator, header);
            }
        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == DepthMapEstimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

#ifdef COMMAND_MODE
void command_process()
{
    while (1)
    {
        // keyboard to start the depth map process
        if (wait2start)
        {
            char c = getchar();
            if (c == 's')
            {
                ROS_INFO("Start to process!");
                start_process = 1;
            }
            wait2start = 0;
        }
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}
#endif //COMMAND_MODE

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_map");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(n);
    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());

    string CORRECT_TOPIC;
    string TD_TOPIC;
    string EXTRINSIC_TOPIC;
    string GRAVITY_TOPIC;
    string CLOUD_TOPIC;
    string FEATURE_TOPIC;
    if (VINS_FRONT_USE)
    {
        ROS_WARN("VINS_MODE_FRONT_END");
        CORRECT_TOPIC = "/vins_estimator/correct_data";
        TD_TOPIC = "/vins_estimator/td";
        EXTRINSIC_TOPIC = "/vins_estimator/extrinsic";
        GRAVITY_TOPIC = "/vins_estimator/gravity";
        CLOUD_TOPIC = "/vins_estimator/vision_local_cloud";
        FEATURE_TOPIC = "/feature_tracker/feature";
    }
    else
    {
        ROS_WARN("LVI_SAM_MODE_FRONT_END");
        CORRECT_TOPIC = "/lvi_sam/vins/odometry/correct_data";
        TD_TOPIC = "/lvi_sam/vins/odometry/td";
        EXTRINSIC_TOPIC = "/lvi_sam/vins/odometry/extrinsic";
        GRAVITY_TOPIC = "/lvi_sam/vins/odometry/gravity";
        CLOUD_TOPIC = "/lvi_sam/vins/odometry/vision_local_cloud";
        FEATURE_TOPIC = "/lvi_sam/vins/feature/feature";
    }
    message_filters::Subscriber<lvio_ros_msgs::CorrectData> sub_correct_data(n, CORRECT_TOPIC, 100);
    message_filters::Subscriber<lvio_ros_msgs::Td> sub_td(n, TD_TOPIC, 100);
    message_filters::Subscriber<nav_msgs::Odometry> sub_ric(n, EXTRINSIC_TOPIC, 100);
    message_filters::Subscriber<geometry_msgs::PointStamped> sub_gravity(n, GRAVITY_TOPIC, 100);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_vision_local_cloud(n, CLOUD_TOPIC, 100);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_imgs(n, FEATURE_TOPIC, 100);

    typedef sync_policies::ApproximateTime<lvio_ros_msgs::CorrectData,
                                           sensor_msgs::PointCloud,
                                           sensor_msgs::PointCloud,
                                           lvio_ros_msgs::Td,
                                           nav_msgs::Odometry,
                                           geometry_msgs::PointStamped>
        VinsPolicy;
    Synchronizer<VinsPolicy> sync_vins(VinsPolicy(1000),
                                       sub_correct_data,
                                       sub_imgs,
                                       sub_vision_local_cloud,
                                       sub_td,
                                       sub_ric,
                                       sub_gravity);
    sync_vins.registerCallback(boost::bind(&vins_handler, _1, _2, _3, _4, _5, _6));

    // Subscribe each Lidar point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_LaserCloudEdgeLast(n, "/laserOdometry/laser_cloud_corner_last", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_LaserCloudFlatLast(n, "/laserOdometry/laser_cloud_surf_last", 100);
    message_filters::Subscriber<nav_msgs::Odometry> sub_LaserOdometry(n, "/laserOdometry/laser_odom_to_init", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_LaserCloudFullRes(n, "/laserOdometry/velodyne_cloud_3", 100);

    // Message filter from ALOAM
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                           sensor_msgs::PointCloud2,
                                           nav_msgs::Odometry,
                                           sensor_msgs::PointCloud2>
        LoamPolicy;
    Synchronizer<LoamPolicy> sync_loam(LoamPolicy(1000),
                                       sub_LaserCloudEdgeLast,
                                       sub_LaserCloudFlatLast,
                                       sub_LaserOdometry,
                                       sub_LaserCloudFullRes);
    sync_loam.registerCallback(boost::bind(&loam_handler, _1, _2, _3, _4));

    std::thread depth_map_thread;
    depth_map_thread = std::thread(depth_map_process);

#ifdef COMMAND_MODE
    std::thread command_thread;
    command_thread = std::thread(command_process);
#endif //COMMAND_MODE

    ros::spin();
    return 0;
}