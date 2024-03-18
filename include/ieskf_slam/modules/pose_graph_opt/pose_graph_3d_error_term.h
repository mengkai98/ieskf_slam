#pragma once
#include "ieskf_slam/type/pose.h"
#include <ceres/ceres.h>
namespace IESKFSlam
{
class PoseGraph3dErrorTerm
{
  public:
    // 构造函数：记录一个约束
    PoseGraph3dErrorTerm(Pose constraint, Eigen::Matrix<double, 6, 6> input_sqrt_information)
        : q_constraint(constraint.rotation), p_constraint(constraint.position),
          sqrt_information(input_sqrt_information){};
    /// @brief 优化过程中计算残差
    /// @tparam T ceres::double 特定类型用于自动求导
    /// @param p_a_ptr : to vertex
    /// @param q_a_ptr
    /// @param p_b_ptr: from_vertex
    /// @param q_b_ptr
    /// @param residuals_ptr 残差
    /// @return
    template <typename T>
    bool operator()(const T *const p_a_ptr, const T *const q_a_ptr, const T *const p_b_ptr, const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_relative = q_a_inverse * q_b;

        Eigen::Matrix<T, 3, 1> p_ab_relative = q_a_inverse * (p_b - p_a);
        Eigen::Quaternion<T> delta_q = q_constraint.cast<T>() * q_ab_relative.conjugate();
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = p_ab_relative - p_constraint.cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
        sqrt_information.cast<T>();
        residuals = sqrt_information.cast<T>() * residuals;
        return true;
        // Eigen::Quaternion
    }
    //写一个Create函数来帮助我们简洁的构建残差块
    static ceres::CostFunction *Create(
        const Pose &constraint,
        const Eigen::Matrix<double, 6, 6> &input_sqrt_information = Eigen::Matrix<double, 6, 6>::Identity())
    {
        return new ceres::AutoDiffCostFunction // 使用自动求导
            <PoseGraph3dErrorTerm, 6, 3, 4, 3,
             4> // 6：残差有6维 3：第一个对应p_a_ptr，表示该指针指向的待优化变量的大小为3维，第二个指向p_b_ptr 4:同理
                // 指向四元数
            (new PoseGraph3dErrorTerm(constraint, input_sqrt_information));
    }

  private:
    Eigen::Quaterniond q_constraint;
    Eigen::Vector3d p_constraint;
    Eigen::Matrix<double, 6, 6> sqrt_information;
};

} // namespace IESKFSlam
