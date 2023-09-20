/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 13:00:25
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:03:10
 */
#pragma once 
#include "ieskf_slam/modules/module_base.h"
#include <Eigen/Dense>
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/math/SO3.h"
namespace IESKFSlam
{
    class IESKF: private ModuleBase
    {
    public:
        using Ptr = std::shared_ptr<IESKF>;
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
        class CalcZHInterface
        {
        public:
            virtual bool calculate(const State18&state,Eigen::MatrixXd & Z,Eigen::MatrixXd & H)=0;
        };

        std::shared_ptr<CalcZHInterface> calc_zh_ptr;
    private:
        State18 X;
        Eigen::Matrix<double,18,18> P;
        Eigen::Matrix<double,12,12> Q;
        int iter_times = 10;
        
    public:
        IESKF(const std::string & config_path,const std::string &prefix);
        ~IESKF();
        void predict(IMU imu,double dt);
        bool update();
        const State18&getX();
        void setX(const State18&x_in);
        Eigen::Matrix<double,18,1> getErrorState18(const State18 &s1, const  State18 &s2);
    };
    

    
} // namespace IESKFSlam

