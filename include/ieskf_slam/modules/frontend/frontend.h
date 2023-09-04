/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-09 00:05:03
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:24:34
 */
#pragma once
#include <fstream>

#include "ieskf_slam/globaldefine.h"
#include "ieskf_slam/modules/frontbackPropagate/frontback_propagate.h"
#include "ieskf_slam/modules/frontend/lio_zh_model.h"
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/modules/map/rect_map_manager.h"
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/measure_group.h"

namespace IESKFSlam {
    class FrontEnd : private ModuleBase {
       public:
        using Ptr = std::shared_ptr<FrontEnd>;

       private:
        std::deque<IMU> imu_deque;
        std::deque<Frame> frame_deque;
        std::shared_ptr<IESKF> ieskf_ptr;
        std::shared_ptr<RectMapManager> map_ptr;
        std::shared_ptr<FrontbackPropagate> fbpropagate_ptr;

        std::shared_ptr<std::deque<Frame>> frame_buffer_ptr;

        VoxelFilter voxel_filter;
        LIOZHModel::Ptr lio_zh_model_ptr;
        PCLPointCloudPtr filter_point_cloud_ptr;
        bool imu_inited = false;
        double imu_scale = 1;
        bool enable_record = false;
        std::string record_file_name;
        std::fstream record_file;

       public:
        FrontEnd(const std::string &config_file_path, const std::string &prefix,
                 std::shared_ptr<std::deque<Frame>> iframe_buffer_ptr);
        ~FrontEnd();
        // 需要向前端传入imu和点云数据
        void addImu(const IMU &imu);
        void addPointCloud(const Frame &f);

        // 跟踪
        bool track();
        // 点云读取
        const PCLPointCloud &readCurrentPointCloud();
        const PCLPointCloud &readCurrentLocalMap();
        bool syncMeasureGroup(MeasureGroup &mg);
        void initState(MeasureGroup &mg);
        IESKF::State18 readState();
    };
}  // namespace IESKFSlam
