/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-09 00:47:23
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 00:51:21
 */
#pragma once
#include "common_lidar_process_interface.h"
namespace avia_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t  ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(avia_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
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
                
            }
            return true;
        }
    };

    
} // namespace ROSNoetic
