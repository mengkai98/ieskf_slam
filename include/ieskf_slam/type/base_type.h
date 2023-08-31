/*
 * @Author: Danny 986337252@qq.com
 * @Date: 2023-06-19 13:19:46
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-06-23 17:13:34
 * @FilePath: /ieskf_slam/include/ieskf_slam/type/base_type.h
 */
#pragma once
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "ieskf_slam/type/frame.h"
namespace IESKFSlam {
    // 体素滤波器
    using VoxelFilter = pcl::VoxelGrid<Point>;
    // KDTree
    using KDTree = pcl::KdTreeFLANN<Point>;
    using KDTreePtr = KDTree::Ptr;
    using KDTreeConstPtr = KDTree::ConstPtr;

    // 定义重力常量
    const double GRAVITY = 9.81;
    template <typename _first, typename _second, typename _thrid>
    struct triple {
        _first first;
        _second second;
        _thrid thrid;
    };
}  // namespace IESKFSlam
