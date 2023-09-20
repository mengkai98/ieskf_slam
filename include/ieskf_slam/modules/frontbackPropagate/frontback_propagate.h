/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-18 20:10:32
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-06-19 13:20:53
 */
#pragma once 
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/measure_group.h"
namespace IESKFSlam
{
    class FrontbackPropagate
    {
    private:
    public:
        double imu_scale;
        IMU last_imu;
        FrontbackPropagate(/* args */);
        ~FrontbackPropagate();
        void propagate(MeasureGroup&mg,IESKF::Ptr ieskf_ptr);
    };


} // namespace IESKFSlam
