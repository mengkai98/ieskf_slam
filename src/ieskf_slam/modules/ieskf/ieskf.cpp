/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 16:43:29
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-14 12:14:08
 */
#include "ieskf_slam/modules/ieskf/ieskf.h"
namespace IESKFSlam
{
    IESKF::IESKF(const std::string & config_path,const std::string &prefix):ModuleBase(config_path,prefix,"IESKF")
    {

    }
    
    IESKF::~IESKF()
    {
    }
    void IESKF::predict(const IMU&imu,double dt){

    }
    bool IESKF::update(){
        
    }
    const IESKF::State18&IESKF::getX(){
        return X;
    }
    void IESKF::setX(const IESKF::State18&x_in){
        X = x_in;
    }
} // namespace IESKFSlam
