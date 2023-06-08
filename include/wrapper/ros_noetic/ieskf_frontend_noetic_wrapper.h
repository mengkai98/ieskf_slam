/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 21:04:57
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 00:59:39
 */
#pragma once
#include "ieskf_slam/modules/frontend/frontend.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "wrapper/ros_noetic/lidar_process/avia_process.h"
namespace ROSNoetic
{
    enum LIDAR_TYPE{
        AVIA = 0,

    };
    class IESKFFrontEndWrapper
    {
    private:
        IESKFSlam::FrontEnd::Ptr front_end_ptr;
        ros::Subscriber cloud_subscriber;
        ros::Subscriber imu_subscriber;
        std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;

        void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg);
        void imuMsgCallBack(const sensor_msgs::ImuPtr &msg);
    public:
        IESKFFrontEndWrapper(ros::NodeHandle &nh);
        ~IESKFFrontEndWrapper();
    
    };
    

    
} // namespace ROSNoetic
