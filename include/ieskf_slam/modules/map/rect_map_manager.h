/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 17:28:53
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:03:06
 */
#pragma once 
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/base_type.h"
#include "pcl/common/transforms.h"
#include "ieskf_slam/math/math.h"
namespace IESKFSlam
{
    class RectMapManager :private ModuleBase
    {
    private:
        PCLPointCloudPtr local_map_ptr;
        KDTreePtr kdtree_ptr;
        float map_side_length_2; //正方体地图边长的一半
        float map_resolution;
    public:

        RectMapManager(const std::string &config_file_path,const std::string & prefix );
        ~RectMapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t);
        PCLPointCloudConstPtr getLocalMap(); 
        KDTreeConstPtr readKDtree();
    };
    

} // namespace IESKFSlam
