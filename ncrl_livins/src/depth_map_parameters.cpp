#include "ncrl_livins/depth_map_parameters.h"

std::string IMU_TOPIC;
// int INIT_SIZE;
int ESTIMATE_LC_TD;
int ALIGN_CAMERA_TIME;

// accelerometer measurement noise standard deviation.
double ACC_N, ACC_W;
// gyroscope measurement noise standard deviation.
double GYR_N, GYR_W;
// accelerometer bias random work noise standard deviation.

double lineRes = 0.3;
double planeRes = 0.6;

Eigen::Vector3d G{0.0, 0.0, 9.805};

// Parameters
double LIDAR_TD;
double LC_TD;

Eigen::Vector3d ER_I2L{0, 0, 0};
Eigen::Vector3d ET_I2L{0, 0, 0};

// ncrl_tf::Trans EXTRINSIC; // update in estimator_node

double UPWARD_ANGLE;
double DOWNWARD_ANGLE;
double CW_ANGLE;
double CCW_ANGLE;
bool VINS_FRONT_USE;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
#ifdef GDB_MODE
    std::string config_file = "$(find feature_tracker)/../config/depth_map/move_it_lvi.yaml";
#else
    std::string config_file;
    VINS_FRONT_USE = readParam<bool>(n,"vins_front_use");
    config_file = readParam<std::string>(n, "config_file");
#endif // GDB_MODE
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

    // input as degree and meter
    ER_I2L.x() = fsSettings["EI2L_RX"];
    ER_I2L.y() = fsSettings["EI2L_RY"];
    ER_I2L.z() = fsSettings["EI2L_RZ"];
    ET_I2L.x() = fsSettings["EI2L_TX"];
    ET_I2L.y() = fsSettings["EI2L_TY"];
    ET_I2L.z() = fsSettings["EI2L_TZ"];

    lineRes = fsSettings["lineRes"];
    planeRes = fsSettings["planeRes"];

    // Add: input the extrinsic Lidar-Camera opencv matrix
    cv::Mat cv_R, cv_T;
    fsSettings["extrinsicRotation"] >> cv_R;
    fsSettings["extrinsicTranslation"] >> cv_T;

    ESTIMATE_LC_TD = fsSettings["estimate_lidar_camera_time_difference"];
    ALIGN_CAMERA_TIME = fsSettings["align_camera_time"];
    LIDAR_TD = fsSettings["lidar_td"];
    LC_TD = fsSettings["lidar_camera_time_difference"];
    if (ESTIMATE_LC_TD)
        ROS_INFO_STREAM("only estimate the td and depth of features");
    else
        ROS_INFO_STREAM("transfo to align lidar point cloud to camera frame");

    UPWARD_ANGLE = fsSettings["limited_upward_angle"];
    DOWNWARD_ANGLE = fsSettings["limited_downward_angle"];
    CW_ANGLE = fsSettings["limited_CW_angle"];
    CCW_ANGLE = fsSettings["limited_CCW_angle"];

    fsSettings.release();
}