/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-09 00:31:32
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 00:48:02
 */
#pragma once
#include <sensor_msgs/PointCloud2.h>

#include "ieskf_slam/type/base_type.h"
#include "pcl_conversions/pcl_conversions.h"
namespace ROSNoetic {
    class CommonLidarProcessInterface {
       public:
        // 根据不同的lidar 转换成统一的cloud
        virtual bool process(const sensor_msgs::PointCloud2 &msg, IESKFSlam::Frame &f) = 0;
    };
}  // namespace ROSNoetic
