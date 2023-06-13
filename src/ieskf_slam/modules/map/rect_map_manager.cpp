/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 17:49:23
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-13 17:58:15
 */
#include "ieskf_slam/modules/map/rect_map_manager.h"
#include "pcl/common/transforms.h"
namespace IESKFSlam
{
    RectMapManager::RectMapManager(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"RectMapManager")
    {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
    }
    
    RectMapManager::~RectMapManager()
    {
    }
    void RectMapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t){
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan,scan,compositeTransform(att_q,pos_t).cast<float>());
        *local_map_ptr+=scan;
    }
    void RectMapManager::reset(){
        local_map_ptr->clear();
    }
} // namespace IESKFSlam
