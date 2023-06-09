/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-09 08:42:00
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 08:43:15
 */
#pragma once
#include <Eigen/Dense>
#include "timestamp.h"
namespace IESKFSlam
{
    struct Pose
    {
        TimeStamp time_stamp;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
    };
} // namespace IESKFSlam
