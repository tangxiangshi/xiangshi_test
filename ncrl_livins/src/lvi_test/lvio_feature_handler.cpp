#include "ncrl_livins/lvio_feature_handler.h"

FeaturePerSweep::FeaturePerSweep():
  laserCloudCornerLast(new pcl::PointCloud<PointType>()),
  laserCloudSurfLast(new pcl::PointCloud<PointType>()),
  laserCloudFullRes(new pcl::PointCloud<PointType>()),
  laserCloudCornerStack(new pcl::PointCloud<PointType>()),
  laserCloudSurfStack(new pcl::PointCloud<PointType>())
{
    initial();
}

void FeaturePerSweep::initial()
{
    id = -1;
    // laser Mapping
    laserCloudCenWidth = 10;
    laserCloudCenHeight = 10;
    laserCloudCenDepth = 5;

    // laserMapping
    laserCloudCornerStackNum = -1;
    laserCloudSurfStackNum = -1;
    laserCloudValidNum = -1;
    laserCloudSurroundNum = -1;

    ncrl_tf::setTransFrame("IMU_INIT", "IMU", Tw2curr_I);
    ncrl_tf::setTransFrame("LIDAR_INIT", "LIDAR", Tw2curr_L);
    ncrl_tf::setTransFrame("WORLD", "IMU", Tw2curr_W);

    ncrl_tf::setTransFrame("IMU_ODOM_START", "IMU", Tinit2odom_st_I);
    ncrl_tf::setTransFrame("LIDAR_ODOM_START", "LIDAR", Tinit2odom_st_L);
    ncrl_tf::setTransFrame("IMU_INIT", "IMU_ODOM_START", Tcurr2odom_I);
    ncrl_tf::setTransFrame("LIDAR_INIT", "LIDAR_ODOM_START", Tcurr2odom_L);

    ncrl_tf::setTransFrame("LIDAR_ODOM", "LIDAR", Tinit2odom_L);
    ncrl_tf::setTransFrame("IMU_ODOM", "IMU", Tinit2odom_I);
    Eigen::Quaterniond identityQ(1,0,0,0);
    Eigen::Vector3d zeroV(0,0,0);
    ncrl_tf::setTrans(identityQ, zeroV, Tw2curr_I);
    ncrl_tf::setTrans(identityQ, zeroV, Tw2curr_L);
    ncrl_tf::setTrans(identityQ, zeroV, Tw2curr_W);
    ncrl_tf::setTrans(identityQ, zeroV, Tcurr2odom_L);
    ncrl_tf::setTrans(identityQ, zeroV, Tcurr2odom_I);
    ncrl_tf::setTrans(identityQ, zeroV, Tinit2odom_L);
    ncrl_tf::setTrans(identityQ, zeroV, Tinit2odom_I);
    ncrl_tf::setTrans(identityQ, zeroV, Tinit2odom_st_I);
    ncrl_tf::setTrans(identityQ, zeroV, Tinit2odom_st_L);
}

laser_feature_handler::laser_feature_handler():
  laserCloudCornerFromMap(new pcl::PointCloud<PointType>()),
  laserCloudSurfFromMap(new pcl::PointCloud<PointType>()),
  laserCloudSurround(new pcl::PointCloud<PointType>()),
  laserCloudRegisted(new pcl::PointCloud<PointType>()),
  laserCloudStatic(new pcl::PointCloud<PointType>()),
  kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>()),
  kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>()),
  kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()),
  kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>())
{
    t_trans_end = -1;
    count = 0;

    // laserMapping
    laserCloudCornerFromMapNum = 0;
    laserCloudSurfFromMapNum = 0;
    laserCloudValidNum = 0;
    laserCloudSurroundNum = 0;

    for (int i = 0; i < laserCloudNum; i++)
    {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    }

    Eigen::Quaterniond identityQ(1,0,0,0);
    Eigen::Vector3d zeroV(0,0,0);
    ncrl_tf::setTransFrame("IMU_ODOM", "IMU_ODOM_START",initGuess_I);
    ncrl_tf::setTransFrame("LIDAR_ODOM", "LIDAR_ODOM_START",initGuess_L);
    ncrl_tf::setTrans(identityQ, zeroV, initGuess_I);
    ncrl_tf::setTrans(identityQ, zeroV, initGuess_L);
}