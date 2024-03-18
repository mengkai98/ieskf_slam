#pragma once
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/modules/pose_graph_opt/pose_graph_opt.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/pose.h"
#include <pcl/registration/icp.h> // 包含这个头文件要放在 Scancontext前面，不然会报错

#include "scan_context/Scancontext.h"

namespace IESKFSlam
{
class BackEnd : private ModuleBase
{
  private:
    SC2::SCManager sc_manager;

    Eigen::Quaterniond extrin_r;
    Eigen::Vector3d extrin_t;
    // . 关键帧位姿：
    std::vector<Pose> poses;

    std::vector<PCLPointCloud> clouds;
    std::vector<BinaryEdge> binary_edges;
    VoxelFilter voxel_filter;
    PoseGraphOpt pgo;

  public:
    using Ptr = std::shared_ptr<BackEnd>;

    BackEnd(const std::string &config_file_path, const std::string &prefix);
    bool addFrame(PCLPointCloud &opt_map, PCLPointCloud &cloud, Pose &pose);
    bool scanRegister(Eigen::Matrix4f &match_result, int from_id, int to_id, float angle);
};
} // namespace IESKFSlam