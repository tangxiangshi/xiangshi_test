#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace ros;
nav_msgs::Path gt_path;
Publisher pub_gt_path;

void gtCallback(const geometry_msgs::PoseStampedConstPtr &gt_msg)
{
    geometry_msgs::PoseStamped gt_pose_stamped;
    gt_pose_stamped.header = gt_msg->header;
    gt_pose_stamped.header.frame_id = "world";
    gt_pose_stamped.pose = gt_msg->pose;

    gt_path.header = gt_msg->header;
    gt_path.header.frame_id = "world";
    gt_path.poses.push_back(gt_pose_stamped);
    pub_gt_path.publish(gt_path);
}

int	main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_truth");
    ros::NodeHandle nh;

    Subscriber gt_sub = nh.subscribe("/vrpn_client_node/RigidBody7/pose", 10, gtCallback);    
    pub_gt_path = nh.advertise<nav_msgs::Path>("/ground_truth_path", 10);

    ros::spin();

    return 0;
}
