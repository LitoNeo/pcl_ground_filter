//
// Created by sin on 2020/8/26.
//

#include "ground_filter_core.h"
#include <unistd.h>

#include <utility>

bool endsWith(string str, string suffix){
    if(str.empty()) return false;
    if(str.size() < suffix.size()) return false;
    if(str.substr(str.size()-suffix.size(), suffix.size()) == suffix) return true;
    return false;
}

RayGroundFilter::RayGroundFilter(){
    const string config_file = "config.ini";
    assert(access(config_file.c_str(), 0) == 0);
    config.ReadConfig("config.ini");
    init_param();
}
RayGroundFilter::~RayGroundFilter() {}

bool RayGroundFilter::init_param(){
    // [LIDAR]
    lidar_frame = config.ReadString("LIDAR", "LidarFrame", "");
    lidar_topic = config.ReadString("LIDAR", "LidarTopic", "");
    SENSOR_MODEL = config.ReadInt("LIDAR", "LidarModel", 0);
    SENSOR_HEIGHT = config.ReadFloat("LIDAR", "LidarHeight", 0.);
    // [PARAMS]
    local_max_slope_ = config.ReadFloat("PARAMS", "LocalMaxSlope", 0.);
    general_max_slope_ = config.ReadFloat("PARAMS", "GeneralMaxSlope", 0.);
    min_height_threshold_ = config.ReadFloat("PARAMS", "MinHeightThreshold", 0.);
    reclass_distance_threshold_ = config.ReadFloat("PARAMS", "ReclassDistanceThreshold", 0.);
    RADIAL_DIVIDER_ANGLE = config.ReadFloat("PARAMS", "RadialDividerAngle", 0.);
    concentric_divider_distance_ = config.ReadFloat("PARAMS", "ConcentricDividerDistance", 0.);
    MIN_DISTANCE = config.ReadFloat("PARAMS", "MinDistance", 0.);
    if(SENSOR_MODEL == 0 || MIN_DISTANCE < 0.1){
        // 参数配置错误,直接panic
        exit(1);
    }
    return true;
}

void RayGroundFilter::clip_above(double clip_height,
        const pcl::PointCloud<Point>::Ptr in,
        const pcl::PointCloud<Point>::Ptr out,
        const pcl::PointCloud<Point>::Ptr& above_points
){
    pcl::ExtractIndices<Point> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for // #pragma omp for语法OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速的效果。
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);

    above_points->points.clear();
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false); // false to save the indices
    cliper.filter(*above_points);
}

void RayGroundFilter::remove_close_pt(double min_distance, const pcl::PointCloud<Point>::Ptr in,
                                      const pcl::PointCloud<Point>::Ptr out)
{
    pcl::ExtractIndices<Point> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
        // // extract ROI space  -- but ray ground filter can work well at full space --if your calculate-power is limited, set ROI here
        // double x = in->points[i].x;
        // double y = in->points[i].y;
        // double z = in->points[i].z;
        // if (minX > x || x > maxX || minY > y || y > maxY) {
        //     indices.indices.push_back(i);
        // }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}
//void RayGroundFilter::remove_close_pt_rectangle(double remove_min_x,
//                                                double remove_min_y,
//                                                double remove_max_x,
//                                                double remove_max_y,
//                                                const pcl::PointCloud<Point>::Ptr in,
//                                                const pcl::PointCloud<Point>::Ptr out)
//{
//    pcl::ExtractIndices<Point> cliper;
//
//    cliper.setInputCloud(in);
//    pcl::PointIndices indices;
//#pragma omp for
//    for (size_t i = 0; i < in->points.size(); i++)
//    {
//
//        double x = in->points[i].x;
//        double y = in->points[i].y;
//        double z = in->points[i].z;
//        if (remove_min_x < x && x < remove_max_x && remove_min_y < y && y < remove_max_y)
//        {
//            indices.indices.push_back(i);
//        }
//    }
//    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
//    cliper.setNegative(true); //ture to remove the indices
//    cliper.filter(*out);
//}
/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void RayGroundFilter::XYZI_to_RTZColor(const pcl::PointCloud<Point>::Ptr in_cloud,
                                       PointCloudXYZIRTColor &out_organized_points,
                                       std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                       std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZIRTColor new_point;
        auto radius = (float)sqrt(
                in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;

        //radial divisions更加角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);

        out_radial_ordered_clouds[radial_div].push_back(new_point);

    } //end for

    //将同一根射线上的点按照半径（距离）排序
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
    }
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void RayGroundFilter::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                                  pcl::PointIndices &out_ground_indices,
                                  pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance < concentric_divider_distance_ || height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                //check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ && (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

void RayGroundFilter::point_cb(const pcl::PointCloud<Point>::Ptr &current_pc_ptr,
        pcl::PointCloud<Point>::Ptr &no_ground_cloud_ptr)
{
    pcl::PointCloud<Point>::Ptr cliped_pc_ptr(new pcl::PointCloud<Point>);

    // 高度截取
    pcl::PointCloud<Point>::Ptr above_points(new pcl::PointCloud<Point>());
    clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr, above_points);

    // 过滤过近噪声点
    pcl::PointCloud<Point>::Ptr remove_close(new pcl::PointCloud<Point>);
    remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close); // 过滤过近点
    // remove_close_pt(remove_min_x, remove_min_y, remove_max_x, remove_max_y, cliped_pc_ptr, remove_close);

    PointCloudXYZIRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

    XYZI_to_RTZColor(remove_close, organized_points,
                     radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;

    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

    // pcl::PointCloud<Point>::Ptr ground_cloud_ptr(new pcl::PointCloud<Point>);

    pcl::ExtractIndices<Point> extract_ground;
    extract_ground.setInputCloud(remove_close);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);

    *no_ground_cloud_ptr += *above_points; // 重载过的 +=: 如果使用points.insert(),则必须更新点云的width: PointCloud.resize(xxx)
}


void RayGroundFilter::save(const pcl::PointCloud<Point>::Ptr &cloud, const string& filename) {
    pcl::io::savePCDFile<Point>(filename, *cloud);
}

void RayGroundFilter::DoFilter(const vector<float> &input,
        vector<float> &output,
        const string& outOriginPcdFileName,
        const string& outFilteredPcdFileName) {
    if(input.size() < 3){
        return;
    }
    pcl::PointCloud<Point>::Ptr in(new pcl::PointCloud<Point>());
    in->reserve(input.size()/3);
    pcl::PointCloud<Point>::Ptr out(new pcl::PointCloud<Point>());

    // 构造输入
    int cnt = 0;
    while(cnt+2 < input.size()){
        Point point;
        point.x = input[cnt++];
        point.y = input[cnt++];
        point.z = input[cnt++];
        in->push_back(point);
    }
    save(in, outOriginPcdFileName);  // 保存原始点云为pcd

    // 去地面处理
    point_cb(in, out);

    // 处理输出
    if(out->empty()){
        return;
    }
    output.reserve(3*out->size());
    for(const Point point: out->points){
        output.push_back(point.x);
        output.push_back(point.y);
        output.push_back(point.z);
    }
    save(out, outFilteredPcdFileName);
}


