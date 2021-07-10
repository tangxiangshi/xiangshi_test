#ifndef __POINT_MANAGER__H__
#define __POINT_MANAGER__H__

#include <map>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <lvio_ros_msgs/PointCloud3.h>
#include <lvio_ros_msgs/CorrectData.h>
//#include "ncrl_livins/depth_map_parameters.h"
//#include "utility/point_utility.h"
//#include "utility/ncrl_tf.h"

//using namespace Eigen;
//using namespace std;

//class PointPerFrame
//{
//public:
//    PointPerFrame() = delete; // default
//    PointPerFrame(const lvio_ros_msgs::PointCloud3 img,
//                  const sensor_msgs::PointCloudConstPtr vision_cloud,
//                  const LaserType laser_msg);

//    void inputImage(const lvio_ros_msgs::PointCloud3 img);
//    void inputVinsPtCloud(const sensor_msgs::PointCloudConstPtr vision_cloud);

//    // vins feature points
//    map<int, Matrix<double, 7, 1>> image;
//    PtMap f_pts_w;
//    PtMap f_pts_bj;
//    PtMap f_pts_lj;

//    // Output to rviz
//    PtMap tmp_f_pts_lj;

//    // loam flat points
//    map<int, vector<Vector3d>> flat_pts_l; //output of depth association
//    PtCloud laser_cloud_corner;
//    PtCloud laser_cloud_flat;
//    PtCloud laser_cloud_full;

//    PtCloud local_map_corner;
//    PtCloud local_map_flat;
//};

//class FramePose
//{
//public:
//    FramePose() = delete; // default
//    FramePose(const lvio_ros_msgs::CorrectDataConstPtr vins_pose_speed, const nav_msgs::OdometryConstPtr &laser_pose);

//    double time; // default: time of "sweep time"
    
//    PointUtility::Tf Pose; // VINS pose as initial guess ,and livins pose update
//    PointUtility::Tf pose_vins;
//    PointUtility::Tf pose_aloam; // tranfo_l0_lj
//};

//class PointManager
//{
//public:
//    PointManager(); // default

//    void exParamInit(const VinsType &vins_msg);
//    void alignTransfo(const PointUtility::Tf transfo_cj_c_near);
//    void depthAssociation(const PtCloud &laser_cloud_flat, const PtCloud &vins_cloud);
//    void preprocessing(const VinsType &vins_msg, const LaserType &laser_msg);
//    void guessTransform();
//    void prepareLocalmap();
//    void shiftPtsWindow();
//    void downsampling(PtCloud cloud_in, PtCloud cloud_out, double res);
//    void eraseCloud(int id);
//    void updateGlobalMap();
//    int findUpdateFrameId(const PointType pose_current);
//    vector<Vector3d> getF2FflatCorres(const PointType point);
//    vector<Vector3d> getF2McornerCorres(const PointType point, int frame_id);
//    VectorXd getF2MflatCorres(const PointType point, int frame_id);
//    PtMap guessFeature();

//    int frame_count; // # of frame in window
//    int sweep_id;

//    PointUtility::Tf transfo_cj_c_near;

//    // Information in the window
//    PointPerFrame *pts_frames[(WINDOW_SIZE + 1)];
//    FramePose *poses_frames[(WINDOW_SIZE + 1)];

//    // extrinsic parameter
//    PointUtility::Tf transfo_l_c[(WINDOW_SIZE + 1)];
//    PointUtility::Tf transfo_b_c[(WINDOW_SIZE + 1)];
//    PointUtility::Tf transfo_b_l;

//    // Keyframes choice
//    vector<PtCloud> laserCloudCorner_vec;
//    vector<PtCloud> laserCloudSurface_vec;
//    PtCloud keyframe_position;
//    pcl::PointCloud<PointXYZIT>::Ptr keyframe_time_poses;

//    // Visualization
//    PtCloud vins_cloud_filtered;
    
//    // kdtree search
//    pcl::KdTreeFLANN<PointType>::Ptr keyframes_search;
    
//    pcl::KdTreeFLANN<PointType>::Ptr depth_search_by_flat;
//    pcl::KdTreeFLANN<PointType>::Ptr f2f_search_flat;

//    pcl::KdTreeFLANN<PointType>::Ptr f2m_search_corner;
//    pcl::KdTreeFLANN<PointType>::Ptr f2m_search_flat;
    
//};

#endif
