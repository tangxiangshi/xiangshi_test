#include "vision_test/lvio_parameters.h"

std::string IMU_TOPIC;
int INIT_SIZE;

// accelerometer measurement noise standard deviation.
double ACC_N, ACC_W;
// gyroscope measurement noise standard deviation.
double GYR_N, GYR_W;
// accelerometer bias random work noise standard deviation.

double TD, TR, LC_TD;
double lineRes = 0.3;
double planeRes = 0.6;
double BackgroundDis;

Eigen::Vector3d G{0.0, 0.0, 9.805};
Eigen::Vector3d ER_I2L{0, 0, 0};
Eigen::Vector3d ET_I2L{0, 0, 0};

ncrl_tf::Trans EXTRINSIC; // update in estimator_node
//ncrl_tf::Trans WORLD2IMU; // update in lvio_estimator's initialStructure
//ncrl_tf::Trans IMU2GT;

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
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;
    INIT_SIZE = fsSettings["INITIAL_STRUCTURE_SIZE"];

    // input as degree and meter
    ER_I2L.x() = fsSettings["EI2L_RX"];
    ER_I2L.y() = fsSettings["EI2L_RY"];
    ER_I2L.z() = fsSettings["EI2L_RZ"];
    ER_I2L = deg2rad(ER_I2L);
    ET_I2L.x() = fsSettings["EI2L_TX"];
    ET_I2L.y() = fsSettings["EI2L_TY"];
    ET_I2L.z() = fsSettings["EI2L_TZ"];

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

    lineRes = fsSettings["lineRes"];
    planeRes = fsSettings["planeRes"];
    BackgroundDis = fsSettings["BackgroundDis"];

    LC_TD = fsSettings["lidar_td"];
    
    fsSettings.release();
}
