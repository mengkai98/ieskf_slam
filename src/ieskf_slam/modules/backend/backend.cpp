#include "ieskf_slam/modules/backend/backend.h"

namespace IESKFSlam {
    BackEnd::BackEnd(const std::string &config_file_path, const std::string &prefix,
                     std::shared_ptr<std::deque<Frame>> iframe_buffer_ptr)
        : ModuleBase(config_file_path, prefix, "BackEnd"), frame_buffer_ptr(iframe_buffer_ptr) {
        readParam("freq", freq, 10);
        sc_manager_ptr = std::make_shared<SCManager>(config_file_path, "scan_context");
        print_table();
    }
    BackEnd::~BackEnd() {}
    bool BackEnd::process() {
        std::deque<Frame> &frame_buffer = *frame_buffer_ptr;
        if (frame_buffer.size() - now_frame_id < 1) return false;
        // ScanContext
        sc_manager_ptr->makeAndSaveScancontextAndKeys(*frame_buffer[now_frame_id].cloud_ptr);
        now_frame_id++;
        if (frame_buffer.size() > 50) {
            std::pair<int, float> loop_detect_result = sc_manager_ptr->detectLoopClosureID();
            Pose result;
            if (loop_detect_result.first == -1) return false;
            if (scanToScanMatch(frame_buffer[loop_detect_result.first],
                                frame_buffer[now_frame_id - 1], result,
                                loop_detect_result.second) == false)
                return false;
            BinaryEdge be;
            be.index_a = loop_detect_result.first;
            be.index_b = now_frame_id - 1;
            be.relative_q = result.rotation;
            be.relative_t = result.position;
            binary_edges.push_back(be);
            return CeresOpt::optimizePoseGraph(frame_buffer, binary_edges, 0, now_frame_id - 1);
        }
        return false;
    }
    bool BackEnd::scanToScanMatch(Frame &fa, Frame &fb, Pose &result_pose, float deg) {
        pcl::IterativeClosestPoint<Point, Point> icp_match;
        icp_match.setMaximumIterations(100);

        PCLPointCloudPtr cloud_b_ptr = pcl::make_shared<PCLPointCloud>();
        Eigen::AngleAxisf z_rotation(-deg, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f transform;
        transform.setIdentity();
        transform.block<3, 3>(0, 0) = z_rotation.toRotationMatrix();
        pcl::transformPointCloud(*fb.cloud_ptr, *cloud_b_ptr, transform);
        icp_match.setInputSource(cloud_b_ptr);
        icp_match.setInputTarget(fa.cloud_ptr);
        PCLPointCloud result_cloud;
        icp_match.align(result_cloud);
        transform = fa.pose.toTransformf() * Frame::Extrin.toTransformf() *
                    icp_match.getFinalTransformation() * transform *
                    Frame::Extrin.toTransformf().inverse();
        result_pose.rotation =
            fa.pose.rotation.conjugate() * transform.block<3, 3>(0, 0).cast<double>();
        result_pose.position = fa.pose.rotation.conjugate() *
                               (transform.block<3, 1>(0, 3).cast<double>() - fa.pose.position);
        return icp_match.hasConverged();
    }
    const std::vector<BinaryEdge> &BackEnd::readBinaryEdges() const { return binary_edges; }
    int BackEnd::readNowFrameID() const { return now_frame_id; }
}  // namespace IESKFSlam
   //[Not loop] Nearest distance: 0.189 btn 107 and 42.
   //[Not loop] yaw diff : 18 deg.