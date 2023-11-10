#include "read_g2o.hpp"
#include "pose_graph_3d_error_term.h"
#include <iostream>
std::vector<Pose3D>vertexs;
std::vector<BinaryEdge>binary_edges;
void outputPose(const std::string& path){
    std::fstream outfile(path,std::ios::out);
    if(!outfile)return;
    for (int i = 0; i < vertexs.size(); i++)
    {
        outfile << i << " " << vertexs[i].p.transpose() << " "
            << vertexs[i].q.x() << " " << vertexs[i].q.y() << " "
            << vertexs[i].q.z() << " " << vertexs[i].q.w() << '\n';
    }
    outfile.close();
}
int main(int argc, char const *argv[])
{
    std::cout<<WORK_SPACE_DIR<<std::endl;
    if(readG2O(WORK_SPACE_DIR+"sphere_data.g2o.txt",vertexs,binary_edges)){
        std::cout<<"READ G2O FILE COMPLETED!!"<<std::endl;
        std::cout<<"Vertexs Size: "<<vertexs.size()<<std::endl;
        std::cout<<"Constraints Size: "<<binary_edges.size()<<std::endl;
    }
    outputPose(WORK_SPACE_DIR+"init_pose.txt");
    //*  构建Problem

    ceres::Problem *problem = new ceres::Problem();
    // 核函数, 这里就不用核函数了，当然也可以试试其他的核函数
    ceres::LossFunction* loss_function = nullptr;
    // 四元数流形，指定四元数的求导方式
    ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold();
    //添加二元边约束(添加二元边产生的误差)
    for( const auto &be:binary_edges){

        // 对信息矩阵开方
        Eigen::Matrix<double,6,6>sqrt_information =be.information.llt().matrixL();
        // 构建误差函数

        ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(be.constraint_pose,sqrt_information);
        // 添加残差块，指定优化变量，这里的顺序对应我们计算残差时传入的变量的顺序。
        problem->AddResidualBlock(  cost_function,
                                    loss_function,
                                    vertexs[be.id_a].p.data(),
                                    vertexs[be.id_a].q.coeffs().data(),
                                    vertexs[be.id_b].p.data(),
                                    vertexs[be.id_b].q.coeffs().data()
        );
        //对于四元数，要设定使用四元数流形的求导方式：
        problem->SetManifold(vertexs[be.id_a].q.coeffs().data(),quaternion_manifold);
        problem->SetManifold(vertexs[be.id_b].q.coeffs().data(),quaternion_manifold);
    }
    // 对于第一个顶点，我们希望它固定
    problem->SetParameterBlockConstant(vertexs.front().q.coeffs().data());
    problem->SetParameterBlockConstant(vertexs.front().p.data());

    //* 求解
    
    // 配置求解参数 
    ceres::Solver::Options options;
    // 最大迭代次数
    options.max_num_iterations = 200;
    // 线性求解方式：考虑一下这几种求解方式在应用场景和效率上各有什么不同？
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    // 求解
    ceres::Solve(options,problem,&summary);
    // 输出求解结果
    std::cout<<summary.FullReport()<<std::endl;
    //* 保存结果
    outputPose(WORK_SPACE_DIR+"opt_pose.txt");

    return 0;
}
