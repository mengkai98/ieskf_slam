/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 23:44:40
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 14:58:10
 */
#pragma once
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/impl/pcl_base.hpp>
#include <pcl/pcl_base.h>
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
namespace IESKFSlam { 
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        std::uint32_t offset_time;
        std::int32_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} 
POINT_CLOUD_REGISTER_POINT_STRUCT(IESKFSlam::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::int32_t, ring, ring)
)