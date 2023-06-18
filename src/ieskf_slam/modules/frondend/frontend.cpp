/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-09 00:07:58
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-18 20:28:48
 */
#include "ieskf_slam/modules/frontend/frontend.h"

namespace IESKFSlam
{
    FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"Front End Module")
    {
        ieskf_ptr = std::make_shared<IESKF>(config_file_path,"ieskf");
        map_ptr  = std::make_shared<RectMapManager>(config_file_path,"map");
        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
    }
    
    FrontEnd::~FrontEnd()
    {
    }
    void FrontEnd::addImu(const IMU&imu){
        imu_deque.push_back(imu);
    }
    void FrontEnd::addPointCloud(const PointCloud&pointcloud){
        pointcloud_deque.push_back(pointcloud);
        std::cout<<"receive cloud"<<std::endl;
    }
    bool FrontEnd::track(){
        MeasureGroup mg;
        if(syncMeasureGroup(mg)){
            
            if(!imu_inited){
                map_ptr->reset();
                map_ptr->addScan(mg.cloud.cloud_ptr,Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            fbpropagate_ptr->propagate(mg,ieskf_ptr);
            return true;
        }
        return false;
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud(){
        return current_pointcloud;
    }
    bool FrontEnd::syncMeasureGroup(MeasureGroup&mg){
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();
        if ( pointcloud_deque.empty()||imu_deque.empty())
        {

            return false;
        }
        ///. wait for imu
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time =pointcloud_deque.front().time_stamp.sec();
        double cloud_end_time = pointcloud_deque.front().cloud_ptr->points.back().offset_time/1e9+cloud_start_time;
        
        if (imu_end_time<cloud_end_time){

            return false;
        }
        

        if (cloud_end_time<imu_start_time)
        {

            pointcloud_deque.pop_front();
            return false;
        }
        mg.cloud = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;
        while (!imu_deque.empty())
        {
            if (imu_deque.front().time_stamp.sec()<mg.lidar_end_time)
            {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();
                
            }else{
                break;
            }
        }
        if(mg.imus.size()<=5){

            return false;
        }
        return true;
    }
    void FrontEnd::initState(MeasureGroup&mg){
        static int imu_count = 0;
        static Eigen::Vector3d mean_acc{0,0,0};
        auto &ieskf = *ieskf_ptr;
        if (imu_inited)
        {
            return ;
        }
        
        for (size_t i = 0; i < mg.imus.size(); i++)
        {
            imu_count++;
            auto x = ieskf.getX();
            mean_acc +=mg.imus[i].acceleration;
            x.bg += mg.imus[i].gyroscope;
            ieskf.setX(x);

        }
        if (imu_count >= 5)
        {
            auto x = ieskf.getX();
            mean_acc /=double(imu_count);

            x.bg /=double(imu_count);
            imu_scale  = GRAVITY/mean_acc.norm();
            fbpropagate_ptr->imu_scale = imu_scale;
            fbpropagate_ptr->last_imu = mg.imus.back();
            // 重力的符号为负 就和fastlio公式一致
            x.gravity = - mean_acc / mean_acc.norm() * GRAVITY;
            ieskf.setX(x);
            imu_inited = true;
        }
        return ;
    }
} // namespace IESKFSlam