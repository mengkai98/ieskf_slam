#pragma once
#include <deque>

#include "ieskf_slam/modules/optimizer/opt_type.h"
#include "ieskf_slam/modules/optimizer/pose_graph_3d_error_term.h"
#include "ieskf_slam/type/frame.h"
namespace IESKFSlam {
    namespace CeresOpt {
        static bool optimizePoseGraph(std::deque<Frame>& frames,
                                      const std::vector<BinaryEdge>& binary_edges,
                                      int start_index = 0, int end_index = 0) {
            assert(end_index > start_index && end_index < frames.size() - 1);
            if (binary_edges.empty()) return false;
            ceres::Problem problem;
            ceres::LossFunction* loss_function = nullptr;
            ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
            int valid_edge_num = 0;
            for (const BinaryEdge& binary_edge : binary_edges) {
                if (binary_edge.index_a < start_index || binary_edge.index_a > end_index ||
                    binary_edge.index_b < start_index || binary_edge.index_b > end_index)
                    continue;
                ceres::CostFunction* cost_function =
                    PoseGraph3dErrorTerm::Create(binary_edge.relative_q, binary_edge.relative_t);
                problem.AddResidualBlock(cost_function, loss_function,
                                         frames[binary_edge.index_a].pose.position.data(),
                                         frames[binary_edge.index_a].pose.rotation.coeffs().data(),
                                         frames[binary_edge.index_b].pose.position.data(),
                                         frames[binary_edge.index_b].pose.rotation.coeffs().data());
                problem.SetManifold(frames[binary_edge.index_a].pose.rotation.coeffs().data(),
                                    quaternion_manifold);
                problem.SetManifold(frames[binary_edge.index_b].pose.rotation.coeffs().data(),
                                    quaternion_manifold);
                valid_edge_num++;
            }
            for (int i = start_index; i < end_index; i++) {
                Eigen::Quaterniond rq =
                    frames[i].pose.rotation.conjugate() * frames[i + 1].pose.rotation;
                Eigen::Vector3d rt = frames[i].pose.rotation.conjugate() *
                                     (frames[i + 1].pose.position - frames[i].pose.position);
                ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(rq, rt);
                problem.AddResidualBlock(
                    cost_function, loss_function, frames[i].pose.position.data(),
                    frames[i].pose.rotation.coeffs().data(), frames[i + 1].pose.position.data(),
                    frames[i + 1].pose.rotation.coeffs().data());
                problem.SetManifold(frames[i].pose.rotation.coeffs().data(), quaternion_manifold);
                problem.SetManifold(frames[i + 1].pose.rotation.coeffs().data(),
                                    quaternion_manifold);
            }

            if (valid_edge_num) {
                ceres::Solver::Options options;
                options.max_num_iterations = 200;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                std::cout << summary.BriefReport() << '\n';
                return summary.IsSolutionUsable();
            } else {
                return false;
            }
        }
    }  // namespace CeresOpt

}  // namespace IESKFSlam
