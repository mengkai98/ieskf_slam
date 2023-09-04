#pragma once

#include "ieskf_slam/modules/module_base.h"

#include "ieskf_slam/type/base_type.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/ndt.h"
#include "ieskf_slam/modules/scancontext/Scancontext.h"
#include "ieskf_slam/modules/optimizer/ceres_pg_opt.hpp"
namespace IESKFSlam {

    class BackEnd : public ModuleBase {
       public:
        using Ptr = std::shared_ptr<BackEnd>;

       private:
        int freq = 0;
        int now_frame_id = 0;
        std::shared_ptr<std::deque<Frame>> frame_buffer_ptr;
        std::shared_ptr<SCManager> sc_manager_ptr;
        std::vector<BinaryEdge> binary_edges;

       public:
        BackEnd(const std::string &config_file_path, const std::string &prefix,
                std::shared_ptr<std::deque<Frame>> iframe_buffer_ptr);
        ~BackEnd();
        bool process();
        bool scanToScanMatch(Frame &fa, Frame &fb, Pose &result_pose, float deg);
        const std::vector<BinaryEdge> &readBinaryEdges() const;
        int readNowFrameID() const;
    };

}  // namespace IESKFSlam