/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-09 00:47:23
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-10 00:27:59
 */
#pragma once
#include "common_lidar_process_interface.h"
namespace avia_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t offset_time;
      std::uint8_t  line;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(avia_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::uint8_t, line, line)
)
namespace ROSNoetic
{
    class AVIAProcess :public CommonLidarProcessInterface
    {
    private:
        /* data */
    public:
        AVIAProcess(/* args */){}
        ~AVIAProcess(){}
        bool process(const sensor_msgs::PointCloud2 &msg,IESKFSlam::PointCloud&cloud){
            pcl::PointCloud<avia_ros::Point> avia_cloud;
            pcl::fromROSMsg(msg,avia_cloud);
            cloud.cloud_ptr->clear();
            for (auto &&point : avia_cloud)
            {
                IESKFSlam::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                p.intensity = point.intensity;
                p.ring = point.line;
                p.offset_time = point.offset_time;
                cloud.cloud_ptr->push_back(p);
            }
            cloud.time_stamp.fromNsec(msg.header.stamp.toNSec());
            return true;
        }
    };

    
} // namespace ROSNoetic
