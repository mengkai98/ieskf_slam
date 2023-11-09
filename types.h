#pragma once
#include <Eigen/Dense>
struct Pose3D{
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
};

struct BinaryEdge
{
    size_t id_a;
    size_t id_b;
    Pose3D constraint_pose;
    Eigen::Matrix<double,6,6> information;
};