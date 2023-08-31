#pragma once
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/base_type.h"
namespace IESKFSlam {

    class BackEnd : public ModuleBase {
       private:
        /* data */
       public:
        BackEnd(const std::string &config_file_path, const std::string &prefix);
        ~BackEnd();
        // void process(IESKFSlam::PointCloud &cloud, );
    };

}  // namespace IESKFSlam