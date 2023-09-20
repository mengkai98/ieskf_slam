/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 13:00:25
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-14 12:27:05
 */
#pragma once 
#include "ieskf_slam/modules/module_base.h"
#include <Eigen/Dense>
#include "ieskf_slam/type/imu.h"
namespace IESKFSlam
{
    class IESKF: private ModuleBase
    {
    public:
        struct State18
        {
            Eigen::Quaterniond rotation;
            Eigen::Vector3d position;
            Eigen::Vector3d velocity;
            Eigen::Vector3d bg;
            Eigen::Vector3d ba;
            Eigen::Vector3d gravity;
            State18(){
                rotation = Eigen::Quaterniond::Identity();
                position = Eigen::Vector3d::Zero();
                velocity = Eigen::Vector3d::Zero();
                bg = Eigen::Vector3d::Zero();
                ba = Eigen::Vector3d::Zero();
                gravity = Eigen::Vector3d::Zero();
            }
        };
    private:
        State18 X;
    public:
        IESKF(const std::string & config_path,const std::string &prefix);
        ~IESKF();
        void predict(const IMU&imu,double dt);
        bool update();
        const State18&getX();
        void setX(const State18&x_in);
    };
    

    
} // namespace IESKFSlam

