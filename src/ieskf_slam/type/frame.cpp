#include "ieskf_slam/type/frame.h"
namespace IESKFSlam {
    Pose Frame::Extrin;
    Frame::Frame() { cloud_ptr = pcl::make_shared<PCLPointCloud>(); }
}  // namespace IESKFSlam
