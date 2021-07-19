#ifndef __DEPTH_MAP_UTILITY__H__
#define __DEPTH_MAP_UTILITY__H__

#include <map>
#include <cmath>
#include <cassert>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;

typedef map<int, Vector3d> PtMap;
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType>::Ptr PtCloud;

class PointUtility
{
public:
    PointUtility(); // default

    struct Tf
    {
        // default
        Tf() {}
        // constructor
        Tf(Quaterniond _q, Vector3d _t) : q{_q}, t{_t} {}

        Quaterniond q;
        Vector3d t;
    };

    static Tf inverse(const Tf trans_a_b)
    {
        Tf trans_b_a(trans_a_b.q.inverse(), trans_a_b.q.inverse() * -trans_a_b.t);
        return trans_b_a;
    }

    /**
     * @brief: a is start frame, b is end frame
     */
    static PtMap transfoPtMap2End(const Tf transfo_a_b, const PtMap a_pts)
    {
        PtMap a_pts_b;
        for (auto a_pt = a_pts.begin(); a_pt != a_pts.end(); a_pt++)
            a_pts_b.insert(pair<int, Vector3d>(a_pt->first, transfo_a_b.q.inverse() * (a_pt->second - transfo_a_b.t)));
        
        return a_pts_b;
    }

    /**
     * @brief: find frame-to-frame transformation. any: any-frame(ex: world frame)
     */
    static Tf findF2FTransfo(const Tf transfo_any_a, const Tf transfo_any_b)
    {
        Tf transfo_a_b(transfo_any_a.q.inverse() * transfo_any_b.q,
                       transfo_any_a.q.inverse() * (transfo_any_b.t - transfo_any_a.t));
        return transfo_a_b;
    }

    static Tf addTransfo(const Tf transfo_a_b, const Tf transfo_b_c)
    {
        Tf transfo_a_c(transfo_a_b.q * transfo_b_c.q,
                       transfo_a_b.q * transfo_b_c.t + transfo_a_b.t);
        return transfo_a_c;
    }

    static void filterVerticalRange(PtMap &pts_map, const double limited_upward_angle, const double limited_downward_angle)
    {
        // ROS_DEBUG_STREAM("Origin points size: \t" << pts_map.size() << "\t before filter vertical range");

        for (auto pt = pts_map.cbegin(); pt != pts_map.cend();)
        {
            double angle = atan(pt->second.z() / sqrt(pow(pt->second.x(), 2) + pow(pt->second.y(), 2))) * 180 / M_PI;
            if (angle > limited_upward_angle || angle < limited_downward_angle)
                pts_map.erase(pt++);
            else
                pt++;
        }

        // ROS_DEBUG_STREAM("Current points size: \t" << pts_map.size());
    }

    static void filterYawRange(PtMap &pts_map, const double limited_CW_angle, const double limited_CCW_angle)
    {
        // ROS_DEBUG_STREAM("Origin points size: \t" << pts_map.size() << "\t before filter yaw range");

        for (auto pt = pts_map.cbegin(); pt != pts_map.cend();)
        {
            double angle = atan(pt->second.y() / sqrt(pow(pt->second.x(), 2) + pow(pt->second.z(), 2))) * 180 / M_PI;
            if (angle > limited_CW_angle || angle < limited_CCW_angle)
                pts_map.erase(pt++);
            else
                pt++;
        }

        // ROS_DEBUG_STREAM("Current points size: \t" << pts_map.size());
    }

    /**
     * @brief: from type of map to type of pcl
     */
    static void map2Cloud(const PtMap pts_map, PtCloud &pt_cloud)
    {
        pt_cloud->points.resize(pts_map.size());
        pt_cloud->width = pts_map.size();
        pt_cloud->height = 1;

        int index = 0;
        for (auto pt = pts_map.begin(); pt != pts_map.end(); pt++, index++)
        {
            pt_cloud->points[index].x = pt->second.x();
            pt_cloud->points[index].y = pt->second.y();
            pt_cloud->points[index].z = pt->second.z();
            pt_cloud->points[index].intensity = pt->first; // id
        }
    }

    /**
     * @brief: b is input frame(ex: sensor frame). a is output frame(ex: world frame)
     */
    static void transfoCloud(const Tf transfo_a_b, const PtCloud cloud_in, PtCloud &cloud_out)
    {
        // Project the one sweep
        int pts_num = cloud_in->points.size();
        for (int i = 0; i < pts_num; i++)
        {
            Vector3d point_b(cloud_in->points[i].x,
                             cloud_in->points[i].y,
                             cloud_in->points[i].z);
            Vector3d point_a = transfo_a_b.q * point_b + transfo_a_b.t;

            cloud_out->points[i].x = point_a.x();
            cloud_out->points[i].y = point_a.y();
            cloud_out->points[i].z = point_a.z();
            cloud_out->points[i].intensity = cloud_in->points[i].intensity;
        }
    }
};

#endif //!__DEPTH_MAP_UTILITY__H__