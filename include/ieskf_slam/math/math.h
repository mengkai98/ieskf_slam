/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 17:55:35
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:18:11
 */
#pragma once
#include <Eigen/Dense>
namespace IESKFSlam
{
    static Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond & q,const Eigen::Vector3d & t){
        Eigen::Matrix4d ans;
        ans.setIdentity();
        ans.block<3,3>(0,0) = q.toRotationMatrix();
        ans.block<3,1>(0,3) = t;
        return ans;
    }
} // namespace IESKFSlam
