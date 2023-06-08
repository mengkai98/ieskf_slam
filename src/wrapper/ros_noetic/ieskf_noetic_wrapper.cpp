/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 21:05:55
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 01:05:18
 */
#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include "ieskf_slam/globaldefine.h"

namespace ROSNoetic
{
    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh)
    {
        std::string config_file_name,lidar_topic,imu_topic;
        nh.param<std::string>("wrapper/config_file_name",config_file_name,"");
        nh.param<std::string>("wrapper/lidar_topic",lidar_topic,"/lidar");
        nh.param<std::string>("wrapper/imu_topic",imu_topic,"/imu");
        
        front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR+config_file_name,"front_end");

        // 发布者和订阅者
        cloud_subscriber = nh.subscribe(lidar_topic,100,&IESKFFrontEndWrapper::lidarCloudMsgCallBack,this);
        imu_subscriber = nh.subscribe(imu_topic,100,&IESKFFrontEndWrapper::imuMsgCallBack,this);
        
        // 读取雷达类型
        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type",lidar_type,AVIA);
        if(lidar_type == AVIA){
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        }
        else{
            std::cout<<"unsupport lidar type"<<std::endl;
            exit(100);
        }

    }
    
    IESKFFrontEndWrapper::~IESKFFrontEndWrapper()
    {
    }
    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg){
        IESKFSlam::PointCloud cloud;
        lidar_process_ptr->process(*msg,cloud);
        front_end_ptr->addPointCloud(cloud);
    }
    
    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg){
        IESKFSlam::IMU imu;
        imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
        imu.acceleration = {msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z};
        imu.gyroscope = {msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z};
        front_end_ptr->addImu(imu);
    }
} // namespace ROSNoetic
