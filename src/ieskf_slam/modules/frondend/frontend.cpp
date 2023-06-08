#include "ieskf_slam/modules/frontend/frontend.h"
namespace IESKFSlam
{
    FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"Front End Module")
    {

    }
    
    FrontEnd::~FrontEnd()
    {
    }
    void FrontEnd::addImu(const IMU&imu){
        imu_deque.push_back(imu);
    }
    void FrontEnd::addPointCloud(const PointCloud&pointcloud){
        pointcloud_deque.push_back(pointcloud);
    }
} // namespace IESKFSlam