/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 17:28:53
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-14 12:24:39
 */
#pragma once 
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/pointcloud.h"

namespace IESKFSlam
{
    class RectMapManager :private ModuleBase
    {
    private:
        PCLPointCloudPtr local_map_ptr;
    public:
        RectMapManager(const std::string &config_file_path,const std::string & prefix );
        ~RectMapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t);
        PCLPointCloudConstPtr getLocalMap(); 
    };
    

} // namespace IESKFSlam
