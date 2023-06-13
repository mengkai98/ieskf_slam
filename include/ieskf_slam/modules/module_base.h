/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 23:59:37
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-10 01:21:13
 */
#pragma once
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
namespace IESKFSlam
{
    class ModuleBase
    {
    private:
        YAML::Node config_node;
        std::string name;

    protected:
        /**
         * @param config_path: 配置文件目录
         * @param prefix: 前缀
         * @param module_name: 模块名称 
        */
        ModuleBase(const std::string&config_path,const std::string&prefix, const std::string & module_name = "default"){
            name = module_name;
            if(config_path!=""){
                try{
                    config_node = YAML::LoadFile(config_path);
                }catch (YAML::Exception &e){
                    std::cout<<e.msg<<std::endl;
                }
                
                if(prefix!=""&&config_node[prefix])config_node = config_node[prefix];
            }
        }
        /**
         * @param T
         * @param key: 键值
         * @param val: 读取数据到哪个参数
         * @param default_val: 默认值
        */
        template<typename T>
        void readParam(const std::string &key,T&val,T default_val){
            if(config_node[key]){
                val = config_node[key].as<T>();
            }else{
                val = default_val;
            }
            //std::cout<<name: <<default_val<<std::endl;
        }
    };
} // namespace SlamCraft

