#pragma once
#include <ceres/ceres.h>
namespace IESKFSlam {
    class PoseGraph3dErrorTerm {
       public:
        PoseGraph3dErrorTerm(Eigen::Quaterniond irelative_q, Eigen::Vector3d irelative_t)
            : relative_q(std::move(irelative_q)), relative_t(std::move(irelative_t)) {}

        template <typename T>
        bool operator()(const T* const p_a_ptr, const T* const q_a_ptr, const T* const p_b_ptr,
                        const T* const q_b_ptr, T* residuals_ptr) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

            // Compute the relative transformation between the two frames.
            Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
            Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

            // Represent the displacement between the two frames in the A frame.
            Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

            // Compute the error between the two orientation estimates.
            Eigen::Quaternion<T> delta_q =
                relative_q.template cast<T>() * q_ab_estimated.conjugate();

            // Compute the residuals.
            // [ position         ]   [ delta_p          ]
            // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
            Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
            residuals.template block<3, 1>(0, 0) = p_ab_estimated - relative_t.template cast<T>();
            residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Quaterniond& irelative_q,
                                           const Eigen::Vector3d& irelative_t) {
            return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
                new PoseGraph3dErrorTerm(irelative_q, irelative_t));
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       private:
        const Eigen::Quaterniond relative_q;
        const Eigen::Vector3d relative_t;
    };
}  // namespace IESKFSlam
