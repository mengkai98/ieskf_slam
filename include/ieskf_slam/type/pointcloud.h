#pragma once
#include "ieskf_slam/type/point.h"
#include "ieskf_slam/type/timestamp.h"
namespace IESKFSlam
{
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    struct PointCloud{
        using Ptr = std::shared_ptr<PointCloud>;
        TimeStamp time_stamp;
        PCLPointCloudPtr cloud_ptr;
        PointCloud(){
            cloud_ptr = boost::make_shared<PCLPointCloud>();
        }
    };
} // namespace IESKFSlam
