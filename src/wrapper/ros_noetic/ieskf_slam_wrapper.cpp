/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-08 21:05:55
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:25:59
 */
#include "wrapper/ros_noetic/ieskf_slam_wrapper.h"
#include "visualization_msgs/Marker.h"
namespace ROSNoetic {
    IESKFSlamWrapper::IESKFSlamWrapper(ros::NodeHandle &nh) {
        std::string config_file_name, lidar_topic, imu_topic;
        nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
        frame_buffer_ptr = std::make_shared<std::deque<IESKFSlam::Frame>>();
        front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR + config_file_name,
                                                              "front_end", frame_buffer_ptr);

        back_end_ptr = std::make_shared<IESKFSlam::BackEnd>(CONFIG_DIR + config_file_name,
                                                            "back_end", frame_buffer_ptr);
        // 发布者和订阅者
        cloud_subscriber =
            nh.subscribe(lidar_topic, 100, &IESKFSlamWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe(imu_topic, 100, &IESKFSlamWrapper::imuMsgCallBack, this);
        // 读取雷达类型
        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        if (lidar_type == AVIA) {
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        } else if (lidar_type == VELO) {
            lidar_process_ptr = std::make_shared<VelodyneProcess>();
        } else {
            std::cout << "unsupport lidar type" << std::endl;
            exit(100);
        }
        curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
        path_pub = nh.advertise<nav_msgs::Path>("path", 100);
        local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);
        loop_pub = nh.advertise<visualization_msgs::Marker>("loop_marker", 100);
        global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map", 100);
        // 拉起线程

        back_end_thread = std::thread(&IESKFSlamWrapper::backendProcess, this);

        run();
    }

    IESKFSlamWrapper::~IESKFSlamWrapper() {}
    void IESKFSlamWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg) {
        IESKFSlam::Frame frame;
        lidar_process_ptr->process(*msg, frame);
        front_end_ptr->addPointCloud(frame);
    }

    void IESKFSlamWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg) {
        IESKFSlam::IMU imu;
        imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
        imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y,
                            msg->linear_acceleration.z};
        imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        front_end_ptr->addImu(imu);
    }
    void IESKFSlamWrapper::run() {
        ros::Rate rate(500);
        while (ros::ok()) {
            rate.sleep();
            ros::spinOnce();
            if (front_end_ptr->track()) {
                publishFrontendMsg();
            }
        }
    }
    void IESKFSlamWrapper::publishFrontendMsg() {
        static nav_msgs::Path path;
        auto X = front_end_ptr->readState();
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped psd;
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        path.poses.push_back(psd);
        path_pub.publish(path);
        IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
        pcl::transformPointCloud(
            cloud, cloud, IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());
        // auto cloud =front_end_ptr->readCurrentPointCloud();
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        curr_cloud_pub.publish(msg);

        cloud = front_end_ptr->readCurrentLocalMap();
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        local_map_pub.publish(msg);
    }
    void IESKFSlamWrapper::backendProcess() {
        ros::Rate rate(10);
        while (ros::ok()) {
            if (back_end_ptr->process()) {
                publishBackendMsg();
            }
            rate.sleep();
        }
    }

    void IESKFSlamWrapper::publishBackendMsg() {
        visualization_msgs::Marker msg;
        msg.header.frame_id = "map";
        msg.ns = "loop_marker";
        msg.id = 0;
        msg.type = visualization_msgs::Marker::LINE_LIST;
        msg.scale.x = 0.1;
        msg.color.a = 1.0;
        msg.color.r = 1.0;
        msg.color.g = 1.0;
        auto bes = back_end_ptr->readBinaryEdges();
        for (int i = 0; i < bes.size(); i++) {
            geometry_msgs::Point gpoint;
            gpoint.x = (*frame_buffer_ptr)[bes[i].index_a].pose.position.x();
            gpoint.y = (*frame_buffer_ptr)[bes[i].index_a].pose.position.y();
            gpoint.z = (*frame_buffer_ptr)[bes[i].index_a].pose.position.z();
            msg.points.push_back(gpoint);
            gpoint.x = (*frame_buffer_ptr)[bes[i].index_b].pose.position.x();
            gpoint.y = (*frame_buffer_ptr)[bes[i].index_b].pose.position.y();
            gpoint.z = (*frame_buffer_ptr)[bes[i].index_b].pose.position.z();
            msg.points.push_back({gpoint});
        }
        loop_pub.publish(msg);
        int now_frame_id = back_end_ptr->readNowFrameID();
        IESKFSlam::PCLPointCloud global_map_cloud;
        IESKFSlam::VoxelFilter filter;
        filter.setLeafSize(0.5, 0.5, 0.5);
        for (int i = 0; i < now_frame_id; i++) {
            auto now_frame = (*frame_buffer_ptr)[i];
            IESKFSlam::PCLPointCloud cloud;
            filter.setInputCloud(now_frame.cloud_ptr);
            filter.filter(cloud);
            pcl::transformPointCloud(
                cloud, cloud,
                now_frame.pose.toTransformf() * IESKFSlam::Frame::Extrin.toTransformf());
            global_map_cloud += cloud;
        }
        sensor_msgs::PointCloud2 ros_point_cloud;
        pcl::toROSMsg(global_map_cloud, ros_point_cloud);
        ros_point_cloud.header.frame_id = "map";
        ros_point_cloud.header.stamp = ros::Time::now();
        global_map_pub.publish(ros_point_cloud);
    }
}  // namespace ROSNoetic
