#pragma once
#include <Eigen/Dense>
namespace IESKFSlam {
    struct BinaryEdge {
        int index_a;
        int index_b;
        Eigen::Quaterniond relative_q;
        Eigen::Vector3d relative_t;
    };

}  // namespace IESKFSlam
