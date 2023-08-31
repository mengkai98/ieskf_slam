#include "ieskf_slam/modules/backend/backend.h"
namespace IESKFSlam {
    BackEnd::BackEnd(const std::string &config_file_path, const std::string &prefix)
        : ModuleBase(config_file_path, prefix, "BackEnd") {
        print_table();
    }
    BackEnd::~BackEnd() {}
}  // namespace IESKFSlam