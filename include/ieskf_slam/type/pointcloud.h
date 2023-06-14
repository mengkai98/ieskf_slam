/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 23:47:42
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-14 12:04:45
 */
#pragma once
#include "ieskf_slam/type/point.h"
#include "ieskf_slam/type/timestamp.h"
namespace IESKFSlam
{
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;
    struct PointCloud{
        using Ptr = std::shared_ptr<PointCloud>;
        TimeStamp time_stamp;
        PCLPointCloudPtr cloud_ptr;
        PointCloud(){
            cloud_ptr = pcl::make_shared<PCLPointCloud>();
        }
    };
} // namespace IESKFSlam
