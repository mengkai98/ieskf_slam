#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "back_end_running_node");
    ros::NodeHandle nh;
    std::shared_ptr<ROSNoetic::IESKFBackEndWrapper> back_end_ptr;
    back_end_ptr = std::make_shared<ROSNoetic::IESKFBackEndWrapper>(nh);

    return 0;
}
