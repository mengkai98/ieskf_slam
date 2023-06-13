#pragma once 
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/math/math.h"
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
    };
    

} // namespace IESKFSlam
