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
#include "ieskf_slam/type/pose.h"
#include "ieskf_slam/type/timestamp.h"
namespace IESKFSlam {
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;
    class Frame {
       public:
        using Ptr = std::shared_ptr<Frame>;
        // 外参
        static Pose Extrin;

       public:
        Pose pose;
        TimeStamp time_stamp;
        PCLPointCloudPtr cloud_ptr;
        Frame();
    };

}  // namespace IESKFSlam
