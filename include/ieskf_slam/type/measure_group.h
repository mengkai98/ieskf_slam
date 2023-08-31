#pragma once
#include <deque>

#include "ieskf_slam/type/frame.h"
#include "ieskf_slam/type/imu.h"
namespace IESKFSlam {
    struct MeasureGroup {
        double lidar_begin_time;
        std::deque<IMU> imus;
        Frame frame;
        double lidar_end_time;
    };
}  // namespace IESKFSlam
