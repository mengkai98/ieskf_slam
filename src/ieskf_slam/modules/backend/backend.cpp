#include "ieskf_slam/modules/backend/backend.h"
#include "ieskf_slam/globaldefine.h"

namespace IESKFSlam
{
BackEnd::BackEnd(const std::string &config_file_path, const std::string &prefix)
    : ModuleBase(config_file_path, prefix, "Front End Module")
{
    std::vector<double> extrin_v;
    readParam("extrin_r", extrin_v, std::vector<double>());
    extrin_r.setIdentity();
    extrin_t.setZero();
    if (extrin_v.size() == 9)
    {
        Eigen::Matrix3d extrin_r33;
        extrin_r33 << extrin_v[0], extrin_v[1], extrin_v[2], extrin_v[3], extrin_v[4], extrin_v[5], extrin_v[6],
            extrin_v[7], extrin_v[8];
        extrin_r = extrin_r33;
    }
    else if (extrin_v.size() == 3)
    {
        extrin_r.x() = extrin_v[0];
        extrin_r.y() = extrin_v[1];
        extrin_r.z() = extrin_v[2];
        extrin_r.w() = extrin_v[3];
    }
    readParam("extrin_t", extrin_v, std::vector<double>());
    if (extrin_v.size() == 3)
    {
        extrin_t << extrin_v[0], extrin_v[1], extrin_v[2];
    }
    voxel_filter.setLeafSize(0.5, 0.5, 0.5);
}
bool BackEnd::addFrame(PCLPointCloud &opt_map, PCLPointCloud &cloud, Pose &pose)
{
    // 检查是否符合添加关键帧的条件：
    static int cnt = 0;
    if (cnt > 100 || poses.empty() || (poses.back().position - pose.position).norm() > 1)
    {
        sc_manager.makeAndSaveScancontextAndKeys(cloud);
        clouds.push_back(cloud);
        poses.push_back(pose);
        cnt = 0;
        auto res = sc_manager.detectLoopClosureID();
        // 检测到回环

        if (res.first != -1)
        {
            // 进行匹配
            Eigen::Matrix4f trans_icp;
            Eigen::Matrix4d T_L_I, T_I_L, T_c;
            if (scanRegister(trans_icp, clouds.size() - 1, res.first, res.second))
            {
                // 匹配成功，计算约束
                BinaryEdge be;
                be.from_vertex = cloud.size() - 1;
                be.to_vertex = res.first;
                T_L_I.setIdentity();
                T_L_I.block<3, 3>(0, 0) = extrin_r.toRotationMatrix();
                T_L_I.block<3, 1>(0, 3) = extrin_t;
                // 使用性质进行取逆
                T_I_L.block<3, 3>(0, 0) = extrin_r.conjugate().toRotationMatrix();
                T_I_L.block<3, 1>(0, 3) = extrin_r.conjugate() * extrin_t * (-1.0);
                // 计算T_c
                T_c = T_L_I * trans_icp.cast<double>() * T_I_L;
                be.constraint.rotation = T_c.block<3, 3>(0, 0);
                be.constraint.position = T_c.block<3, 1>(0, 3);
                binary_edges.push_back(be);
                // copy 所有的位姿
                std::vector<Pose> copy_poses = poses;
                if (pgo.slove(copy_poses, binary_edges))
                {
                    opt_map.clear();
                    for (int i = 0; i < clouds.size(); i++)
                    {
                        Eigen::Matrix4f T_I_W, T_L_W;
                        T_I_W.setIdentity();
                        T_I_W.block<3, 3>(0, 0) = copy_poses[i].rotation.cast<float>().toRotationMatrix();
                        T_I_W.block<3, 1>(0, 3) = copy_poses[i].position.cast<float>();
                        T_L_W = T_I_W * T_L_I.cast<float>();
                        PCLPointCloud global_cloud;
                        voxel_filter.setInputCloud(clouds[i].makeShared());
                        voxel_filter.filter(global_cloud);
                        pcl::transformPointCloud(global_cloud, global_cloud, T_L_W);
                        opt_map += global_cloud;
                    }
                    voxel_filter.setInputCloud(opt_map.makeShared());
                    voxel_filter.filter(opt_map);
                    // update pose
                    for (int i = 0; i < poses.size(); i++)
                    {
                        poses[i] = copy_poses[i];
                    }

                    return true;
                }
                else
                {
                    binary_edges.pop_back();
                }
            }
        }
    }
    cnt++;

    return false;
}
bool BackEnd::scanRegister(Eigen::Matrix4f &match_result, int from_id, int to_id, float angle)
{
    // 1 . 计算初始位姿：根据ScanContext的定义：
    Eigen::AngleAxisf init_rotation(-1 * angle, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
    // 角轴变换为旋转矩阵赋值为变换矩阵。
    initial_transform.block<3, 3>(0, 0) = init_rotation.toRotationMatrix();
    // 2. ICP 配准
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(clouds[from_id].makeShared());
    icp.setInputTarget(clouds[to_id].makeShared());
    // 迭代次数
    icp.setMaximumIterations(20);
    PCLPointCloud result;
    icp.align(result, initial_transform);
    // 3.  获取配准结果
    match_result = icp.getFinalTransformation();
    std::cout << "icp score: " << icp.getFitnessScore() << std::endl;
    // 4. 得分小于多少，认为配准成功
    return icp.getFitnessScore() < 3.0;
}
} // namespace IESKFSlam

/**
    [Loop found] Nearest distance: 0.176 btn 272 and 15.
    [Loop found] yaw diff: 228 deg.
    15 | 3.98
*/