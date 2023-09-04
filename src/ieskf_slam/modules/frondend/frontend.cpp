/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-09 00:07:58
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:27:35
 */
#include "ieskf_slam/modules/frontend/frontend.h"
namespace IESKFSlam {
    FrontEnd::FrontEnd(const std::string &config_file_path, const std::string &prefix,
                       std::shared_ptr<std::deque<Frame>> iframe_buffer_ptr)
        : ModuleBase(config_file_path, prefix, "Front End Module"),
          frame_buffer_ptr(iframe_buffer_ptr) {
        float leaf_size;
        readParam("filter_leaf_size", leaf_size, 0.5f);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        std::vector<double> extrin_v;
        readParam("extrin_r", extrin_v, std::vector<double>());
        Frame::Extrin.rotation.setIdentity();
        Frame::Extrin.position.setZero();
        if (extrin_v.size() == 9) {
            Eigen::Matrix3d extrin_r33;
            extrin_r33 << extrin_v[0], extrin_v[1], extrin_v[2], extrin_v[3], extrin_v[4],
                extrin_v[5], extrin_v[6], extrin_v[7], extrin_v[8];
            Frame::Extrin.rotation = extrin_r33;
        } else if (extrin_v.size() == 3) {
            Frame::Extrin.rotation.x() = extrin_v[0];
            Frame::Extrin.rotation.y() = extrin_v[1];
            Frame::Extrin.rotation.z() = extrin_v[2];
            Frame::Extrin.rotation.w() = extrin_v[3];
        }
        readParam("extrin_t", extrin_v, std::vector<double>());
        if (extrin_v.size() == 3) {
            Frame::Extrin.position << extrin_v[0], extrin_v[1], extrin_v[2];
        }
        ieskf_ptr = std::make_shared<IESKF>(config_file_path, "ieskf");
        map_ptr = std::make_shared<RectMapManager>(config_file_path, "map");

        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
        lio_zh_model_ptr = std::make_shared<LIOZHModel>();
        ieskf_ptr->calc_zh_ptr = lio_zh_model_ptr;
        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        lio_zh_model_ptr->prepare(map_ptr->readKDtree(), filter_point_cloud_ptr,
                                  map_ptr->getLocalMap());

        readParam("enable_record", enable_record, false);
        readParam("record_file_name", record_file_name, std::string("default.txt"));
        if (enable_record) {
            record_file.open(RESULT_DIR + record_file_name, std::ios::out | std::ios::app);
        }

        print_table();
    }
    FrontEnd::~FrontEnd() { record_file.close(); }

    void FrontEnd::addImu(const IMU &imu) { imu_deque.push_back(imu); }

    void FrontEnd::addPointCloud(const Frame &frame) {
        frame_deque.push_back(frame);
        pcl::transformPointCloud(*frame_deque.back().cloud_ptr, *frame_deque.back().cloud_ptr,
                                 Frame::Extrin.toTransformf());
    }

    bool FrontEnd::track() {
        MeasureGroup mg;
        static size_t cnt = 0;
        if (syncMeasureGroup(mg)) {
            if (!imu_inited) {
                map_ptr->reset();
                map_ptr->addScan(mg.frame.cloud_ptr, Eigen::Quaterniond::Identity(),
                                 Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            fbpropagate_ptr->propagate(mg, ieskf_ptr);
            voxel_filter.setInputCloud(mg.frame.cloud_ptr);
            voxel_filter.filter(*filter_point_cloud_ptr);
            ieskf_ptr->update();
            auto state = ieskf_ptr->getX();
            if (enable_record) {
                record_file << std::setprecision(15) << mg.lidar_end_time << " "
                            << state.position.x() << " " << state.position.y() << " "
                            << state.position.z() << " " << state.rotation.x() << " "
                            << state.rotation.y() << " " << state.rotation.z() << " "
                            << state.rotation.w() << std::endl;
            }
            map_ptr->addScan(filter_point_cloud_ptr, state.rotation, state.position);
            if (cnt % 10 == 0) {
                Eigen::Matrix4f trans = Frame::Extrin.toTransformf().inverse();
                pcl::transformPointCloud(*mg.frame.cloud_ptr, *mg.frame.cloud_ptr, trans);
                mg.frame.pose.rotation = state.rotation;
                mg.frame.pose.position = state.position;
                frame_buffer_ptr->push_back(mg.frame);
            }
            cnt++;
            return true;
        }
        return false;
    }
    const PCLPointCloud &FrontEnd::readCurrentPointCloud() { return *filter_point_cloud_ptr; }
    const PCLPointCloud &FrontEnd::readCurrentLocalMap() { return *map_ptr->getLocalMap(); }
    bool FrontEnd::syncMeasureGroup(MeasureGroup &mg) {
        mg.imus.clear();
        mg.frame.cloud_ptr->clear();
        if (frame_deque.empty() || imu_deque.empty()) {
            return false;
        }
        ///. wait for imu
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time = frame_deque.front().time_stamp.sec();
        double cloud_end_time =
            frame_deque.front().cloud_ptr->points.back().offset_time / 1e9 + cloud_start_time;

        if (imu_end_time < cloud_end_time) {
            return false;
        }

        if (cloud_end_time < imu_start_time) {
            frame_deque.pop_front();
            return false;
        }
        mg.frame = frame_deque.front();
        frame_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;
        while (!imu_deque.empty()) {
            if (imu_deque.front().time_stamp.sec() < mg.lidar_end_time) {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();

            } else {
                break;
            }
        }
        if (mg.imus.size() <= 5) {
            return false;
        }
        return true;
    }
    void FrontEnd::initState(MeasureGroup &mg) {
        static int imu_count = 0;
        static Eigen::Vector3d mean_acc{0, 0, 0};
        auto &ieskf = *ieskf_ptr;
        if (imu_inited) {
            return;
        }

        for (size_t i = 0; i < mg.imus.size(); i++) {
            imu_count++;
            auto x = ieskf.getX();
            mean_acc += mg.imus[i].acceleration;
            x.bg += mg.imus[i].gyroscope;
            ieskf.setX(x);
        }
        if (imu_count >= 5) {
            auto x = ieskf.getX();
            mean_acc /= double(imu_count);

            x.bg /= double(imu_count);
            imu_scale = GRAVITY / mean_acc.norm();

            // 重力的符号为负 就和fastlio公式一致
            x.gravity = -mean_acc / mean_acc.norm() * GRAVITY;
            ieskf.setX(x);
            imu_inited = true;
            fbpropagate_ptr->imu_scale = imu_scale;
            fbpropagate_ptr->last_imu = mg.imus.back();
            fbpropagate_ptr->last_lidar_end_time_ = mg.lidar_end_time;
        }
        return;
    }
    IESKF::State18 FrontEnd::readState() { return ieskf_ptr->getX(); }
}  // namespace IESKFSlam