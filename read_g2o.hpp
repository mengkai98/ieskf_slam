#pragma once
#include <fstream>
#include "types.h"
// VERTEX_SE3:QUAT ID x y z q_x q_y q_z q_w
// EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 I_13 ... I_16 I_22 I_23 ... I_26 ... I_66 // NOLINT

/// @brief 从G2O文件中读取数据
/// @param file_path 文件的路径
/// @param vertexs 顶点存放的位置
/// @param binary_edges 二元边约束存放的位置
static bool readG2O(const std::string &file_path,std::vector<Pose3D>&vertexs,std::vector<BinaryEdge>&binary_edges){
    
    vertexs.clear();
    binary_edges.clear();
    
    std::fstream g2o_file(file_path,std::ios::in);
    if(!g2o_file)return false;
    
    std::string data_type;
    while (g2o_file.good())
    {
        g2o_file>>data_type;
        if(data_type=="VERTEX_SE3:QUAT"){
            Pose3D pose;
            size_t idx;
            g2o_file>>idx>> pose.p.x() >> pose.p.y() >> pose.p.z() >> 
                            pose.q.x() >> pose.q.y() >> pose.q.z() >> pose.q.w();
            assert(vertexs.size()==idx);
            vertexs.push_back(pose);            
        }
        else if(data_type=="EDGE_SE3:QUAT"){
            BinaryEdge binary_edge;
            g2o_file >> binary_edge.id_a>>binary_edge.id_b>>
                        binary_edge.constraint_pose.p.x() >> binary_edge.constraint_pose.p.y() >> binary_edge.constraint_pose.p.z() >> 
                        binary_edge.constraint_pose.q.x() >> binary_edge.constraint_pose.q.y() >> binary_edge.constraint_pose.q.z() >> binary_edge.constraint_pose.q.w();
            for (int i = 0; i < 6; i++)
            {
                for (int j = i; j < 6; j++)
                {
                    g2o_file >> binary_edge.information(i,j);
                    if(i!=j){
                        binary_edge.information(j,i) = binary_edge.information(i,j);
                    }
                }
                
            }
            binary_edges.push_back(binary_edge);
            
        }else{
            return false;
        }
        
    }
    return true;
}