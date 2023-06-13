/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 16:43:29
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-13 16:43:43
 */
#include "ieskf_slam/modules/ieskf/ieskf.h"
namespace IESKFSlam
{
    IESKF::IESKF(const std::string & config_path,const std::string &prefix):ModuleBase(config_path,prefix,"IESKF")
    {
        float cov_;
        readParam("cov",cov_,1.f);
    }
    
    IESKF::~IESKF()
    {
    }
    void IESKF::predict(const IMU&imu,double dt){

    }
    bool IESKF::update(){
        
    }
} // namespace IESKFSlam
