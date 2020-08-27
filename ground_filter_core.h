//
// Created by sin on 2020/8/26.
//

#ifndef GROUND_FILTER_SHARED_GROUND_FILTER_CORE_H
#define GROUND_FILTER_SHARED_GROUND_FILTER_CORE_H
// #pragma once
#include <pcl/conversions.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//#include <geometry_msgs/PoseStamped.h>
#include <omp.h>
//#include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_listener.h>
#include "config.h"
#include <vector>
using namespace std;

class RayGroundFilter
{
    using Point = pcl::PointXYZI;

private:
    rr::Config config;
    std::string lidar_frame;
    std::string lidar_topic;

    int SENSOR_MODEL;
    double SENSOR_HEIGHT;

    double RADIAL_DIVIDER_ANGLE; // default: 0.18

    double local_max_slope_;   // degree default: 8 //max slope of the ground between points, degree
    double general_max_slope_; // degree  default: 5 //max slope of the ground in entire point cloud, degree

    double CLIP_HEIGHT; //截取掉高于雷达自身xx米的点
    double minX;
    double maxX;
    double minY;
    double maxY;

    // 按照矩形形状,截取掉过近的点,这类点大多是车身
    double remove_min_x;
    double remove_min_y;
    double remove_max_x;
    double remove_max_y;

    // 按照圆形形状,截取掉过近的点,这类点大多是车身
    double MIN_DISTANCE; // default: 2.4

    double concentric_divider_distance_; // default: 0.01 //0.1 meters default
    double min_height_threshold_;        // default: 0.05

    double reclass_distance_threshold_; // default: 0.2

    // DEBUG
    bool debug_pub_all;
    std::string debug_map_topic;

    struct PointXYZIRTColor
    {
        Point point;

        float radius; //cylindric coords on XY Plane
        float theta;  //angle deg on XY plane

        size_t radial_div;     //index of the radial divsion to which this point belongs to
        size_t concentric_div; //index of the concentric division to which this points belongs to

        size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

    size_t radial_dividers_num_;
    size_t concentric_dividers_num_;

    bool init_param();

    void point_cb(const pcl::PointCloud<Point>::Ptr &current_pc_ptr, pcl::PointCloud<Point>::Ptr &no_ground_cloud_ptr);
    // void pose_cb(const geometry_msgs::PoseStampedConstPtr &current_pose_ptr);

    void clip_above(double clip_height,
            const pcl::PointCloud<Point>::Ptr in,
            const pcl::PointCloud<Point>::Ptr out,
            const pcl::PointCloud<Point>::Ptr& above_points
    );

    // 以min_distance为半径,删除过近点  --注:该方式比较粗狂,但也有效. 应保证以雷达中心点为圆心,以min_distance为半径的圆能够覆盖整个车身
    void remove_close_pt(double min_distance, const pcl::PointCloud<Point>::Ptr in, const pcl::PointCloud<Point>::Ptr out);

//    // **以矩形的方式删除过近点,暂时弃用
//    void remove_close_pt_rectangle(double remove_min_x,
//                                   double remove_min_y,
//                                   double remove_max_x,
//                                   double remove_max_y,
//                                   const pcl::PointCloud<Point>::Ptr in,
//                                   const pcl::PointCloud<Point>::Ptr out);

    void XYZI_to_RTZColor(pcl::PointCloud<Point>::Ptr in_cloud,
                          PointCloudXYZIRTColor &out_organized_points,
                          std::vector<pcl::PointIndices> &out_radial_divided_indices,
                          std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

    void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                     pcl::PointIndices &out_ground_indices,
                     pcl::PointIndices &out_no_ground_indices);

    void save(const pcl::PointCloud<Point>::Ptr &cloud, const string& filename);

public:
    RayGroundFilter();
    ~RayGroundFilter();

    void DoFilter(const vector<float> &input,
                  vector<float> &output,
                  const string& outOriginPcdFileName,
                  const string& outFilteredPcdFileName);
};

#endif //GROUND_FILTER_SHARED_GROUND_FILTER_CORE_H
