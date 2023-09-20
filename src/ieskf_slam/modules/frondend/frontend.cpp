/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-09 00:07:58
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-10 00:34:51
 */
#include "ieskf_slam/modules/frontend/frontend.h"
#include "pcl/common/transforms.h"
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
        std::cout<<"receive cloud"<<std::endl;
    }
    void FrontEnd::addPose(const Pose&pose){
        pose_deque.push_back(pose);
        std::cout<<"receive pose"<<std::endl;
    }
    bool FrontEnd::track(){
        if(pose_deque.empty()||pointcloud_deque.empty()){
            return false;
        }
        // 寻找同一时刻的点云和位姿
        
        while (!pose_deque.empty()&&pose_deque.front().time_stamp.nsec()<pointcloud_deque.front().time_stamp.nsec())
        {   
            std::cout<<"1"<<std::endl;
            pose_deque.pop_front();
        }
        if(pose_deque.empty()){
            return false;
        }
        while (!pointcloud_deque.empty()&&pointcloud_deque.front().time_stamp.nsec()<pose_deque.front().time_stamp.nsec())
        {
            std::cout<<"2"<<std::endl;
            pointcloud_deque.pop_front();
        }
        if(pointcloud_deque.empty()){
            return false;
        }
        // 滤波
        VoxelFilter vf;
        vf.setLeafSize(0.5,0.5,0.5);
        vf.setInputCloud(pointcloud_deque.front().cloud_ptr);
        vf.filter(*pointcloud_deque.front().cloud_ptr);

        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3,3>(0,0) = pose_deque.front().rotation.toRotationMatrix().cast<float>();
        trans.block<3,1>(0,3) = pose_deque.front().position.cast<float>();
        pcl::transformPointCloud(*pointcloud_deque.front().cloud_ptr,current_pointcloud,trans);


        pointcloud_deque.pop_front();
        pose_deque.pop_front();
        return true;
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud(){
        return current_pointcloud;
    }
} // namespace IESKFSlam