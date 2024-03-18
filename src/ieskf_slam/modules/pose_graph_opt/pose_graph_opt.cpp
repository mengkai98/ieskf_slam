#include "ieskf_slam/modules/pose_graph_opt/pose_graph_opt.h"
#include "ieskf_slam/modules/pose_graph_opt/pose_graph_3d_error_term.h"
#include <ceres/ceres.h>
namespace IESKFSlam
{
PoseGraphOpt::PoseGraphOpt(/* args */)
{
}

bool PoseGraphOpt::slove(std::vector<Pose> &poses, std::vector<BinaryEdge> &bes)
{
    if (poses.size() <= 10)
        return false;

    // ceres 构建problem
    ceres::Problem *problem = new ceres::Problem();
    // 核函数, 这里就不用核函数了，当然也可以试试其他的核函数
    ceres::LossFunction *loss_function = nullptr;
    // 四元数流形，指定四元数的求导方式
    ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold();
    // 1 添加帧间约束
    for (int i = 0; i < poses.size() - 1; i++)
    {
        Pose constraint;
        // 计算帧间相对位姿，From k+1帧 to k帧，T_c = T_{k}^{-1}T_{k+1} ||
        constraint.rotation = poses[i].rotation.conjugate() * poses[i + 1].rotation;
        constraint.position = poses[i].rotation.conjugate() * (poses[i + 1].position - poses[i].position);

        // 创建残差快
        ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(constraint);
        // 添加残差块
        problem->AddResidualBlock(cost_function, loss_function, poses[i].position.data(),
                                  poses[i].rotation.coeffs().data(), poses[i + 1].position.data(),
                                  poses[i + 1].rotation.coeffs().data());
        //对于四元数，要设定使用四元数流形的求导方式：
        problem->SetManifold(poses[i].rotation.coeffs().data(), quaternion_manifold);
        problem->SetManifold(poses[i + 1].rotation.coeffs().data(), quaternion_manifold);
    }

    // 对于第一个顶点，我们希望它固定
    problem->SetParameterBlockConstant(poses.front().rotation.coeffs().data());
    problem->SetParameterBlockConstant(poses.front().position.data());

    //接下来处理回环约束
    // 比较简单
    for (auto &&be : bes)
    {
        // 创建残差快
        ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(be.constraint);
        // 添加残差块
        problem->AddResidualBlock(cost_function, loss_function, poses[be.to_vertex].position.data(),
                                  poses[be.to_vertex].rotation.coeffs().data(), poses[be.from_vertex].position.data(),
                                  poses[be.from_vertex].rotation.coeffs().data());
        //对于四元数，要设定使用四元数流形的求导方式：
        problem->SetManifold(poses[be.to_vertex].rotation.coeffs().data(), quaternion_manifold);
        problem->SetManifold(poses[be.from_vertex].rotation.coeffs().data(), quaternion_manifold);
    }
    // 配置求解参数
    ceres::Solver::Options options;
    // 最大迭代次数
    options.max_num_iterations = 200;
    // 线性求解方式：考虑一下这几种求解方式在应用场景和效率上各有什么不同？
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    // 求解
    ceres::Solve(options, problem, &summary);
    // 输出求解结果
    std::cout << summary.BriefReport() << std::endl;
    return summary.termination_type == ceres::CONVERGENCE;
}
} // namespace IESKFSlam
