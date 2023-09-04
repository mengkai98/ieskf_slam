#pragma once
#include <Eigen/Eigen>
namespace IESKFSlam {
    struct Pose {
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
        // Eigen::Matrix<double, 6, 6> cov;
        Pose(const Eigen::Quaterniond &ir, const Eigen::Vector3d &ip)
            : rotation(ir), position(ip) {}
        Pose() : rotation(Eigen::Quaterniond::Identity()), position(Eigen::Vector3d::Zero()) {}
        Eigen::Matrix4d toTransformd() {
            Eigen::Matrix4d transform;
            transform.setIdentity();
            transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            transform.block<3, 1>(0, 3) = position;
            return transform;
        };
        Eigen::Matrix4f toTransformf() {
            Eigen::Matrix4d transform;
            transform.setIdentity();
            transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            transform.block<3, 1>(0, 3) = position;
            return transform.cast<float>();
        };
    };

}  // namespace IESKFSlam
