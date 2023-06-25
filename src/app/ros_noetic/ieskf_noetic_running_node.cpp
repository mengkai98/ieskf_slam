/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 21:06:45
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-06-25 14:49:39
 */
#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"front_end_running_node");
    ros::NodeHandle nh;
    std::shared_ptr<ROSNoetic::IESKFFrontEndWrapper>front_end_ptr;
    front_end_ptr = std::make_shared<ROSNoetic::IESKFFrontEndWrapper>(nh);
    return 0;
}
