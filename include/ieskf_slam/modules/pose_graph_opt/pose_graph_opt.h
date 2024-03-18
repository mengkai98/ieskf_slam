#pragma once
#include "ieskf_slam/type/pose.h"
#include <vector>
namespace IESKFSlam
{
struct BinaryEdge
{
    int from_vertex;
    int to_vertex;
    Pose constraint;
    Eigen::Matrix<double, 6, 6> information;
    BinaryEdge(int i_fromv = 0, int i_tov = 0,
               Eigen::Matrix<double, 6, 6> i_infor = Eigen::Matrix<double, 6, 6>::Identity())
        : from_vertex(i_fromv), to_vertex(i_tov), information(i_infor){};
};

class PoseGraphOpt
{
  private:
    /* data */
  public:
    PoseGraphOpt(/* args */);

    bool slove(std::vector<Pose> &poses, std::vector<BinaryEdge> &bes);
};

} // namespace IESKFSlam
