/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 21:05:55
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-06-25 14:48:47
 */
#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
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
        curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud",100);
        path_pub = nh.advertise<nav_msgs::Path>("path",100);

        run();
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
    void IESKFFrontEndWrapper::run(){
        ros::Rate rate(500);
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
            if(front_end_ptr->track()){
                publishMsg();
            }
        }
    }
    void IESKFFrontEndWrapper::publishMsg(){
        static nav_msgs::Path path;
        auto X =  front_end_ptr->readState();
        path.header.frame_id="map";
        geometry_msgs::PoseStamped psd;
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        path.poses.push_back(psd);
        path_pub.publish(path);
        // auto cloud =front_end_ptr->readCurrentPointCloud();
        // sensor_msgs::PointCloud2 msg;
        // pcl::toROSMsg(cloud,msg);
        // msg.header.frame_id = "map";
        // curr_cloud_pub.publish(msg);
    }
} // namespace ROSNoetic
