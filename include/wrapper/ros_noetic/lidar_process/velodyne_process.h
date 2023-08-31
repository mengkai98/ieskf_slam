/*
 * @Descripttion:
 * @Author: Meng Kai
 * @version:
 * @Date: 2023-07-03 20:08:10
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-07-03 21:01:35
 */
#pragma once
#include "wrapper/ros_noetic/lidar_process/common_lidar_process_interface.h"
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time,time)
    (std::uint16_t, ring, ring)
)
namespace ROSNoetic
{
    class VelodyneProcess :public CommonLidarProcessInterface
    {
    private:
    public:
        bool process(const sensor_msgs::PointCloud2 &msg, IESKFSlam::Frame &frame){
            pcl::PointCloud<velodyne_ros::Point> rs_cloud;
            pcl::fromROSMsg(msg,rs_cloud);
            frame.cloud_ptr->clear();
            double end_time = msg.header.stamp.toSec();
            double start_time = end_time + rs_cloud[0].time;
            for (auto &&p : rs_cloud)
            {
                double point_time = p.time +end_time;
                IESKFSlam::Point point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                point.intensity = p.intensity;

                
                point.offset_time = (point_time - start_time)*1e9;
                point.ring = p.ring;
                frame.cloud_ptr->push_back(point);
            
            }
            frame.time_stamp.fromSec(start_time);
            return true;
        }
    };
}