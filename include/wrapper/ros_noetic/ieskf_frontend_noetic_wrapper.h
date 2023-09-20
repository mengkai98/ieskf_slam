/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 21:04:57
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-06-25 14:48:50
 */
#pragma once
#include "ieskf_slam/modules/frontend/frontend.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "wrapper/ros_noetic/lidar_process/avia_process.h"
#include "ieskf_slam/globaldefine.h"
#include "nav_msgs/Path.h"
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
        ros::Publisher curr_cloud_pub;
        ros::Publisher path_pub;
        ros::Publisher local_map_pub;
        std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;

        // now status
        void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg);
        void imuMsgCallBack(const sensor_msgs::ImuPtr &msg);
        void run();
        void publishMsg();
    public:
        IESKFFrontEndWrapper(ros::NodeHandle &nh);
        ~IESKFFrontEndWrapper();

    };
    

    
} // namespace ROSNoetic
