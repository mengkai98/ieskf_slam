/*
 * @Author: Danny 986337252@qq.com
 * @Date: 2023-06-23 17:12:09
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-06-25 14:47:00
 * @FilePath: /ieskf_slam/include/ieskf_slam/math/geometry.hpp
 */
#pragma once
#include <Eigen/Dense>
#include <vector>
namespace IESKFSlam
{
    template< typename pointTypeT >
    bool planarCheck(const std::vector<pointTypeT> & points, Eigen::Vector4d &pabcd, float threhold){
        Eigen::Vector3d normal_vector;
        Eigen::MatrixXd A;
        Eigen::VectorXd B;
        int point_num = points.size();
        A.resize(point_num,3);
        B.resize(point_num);
        B.setOnes();
        B = -1*B;
        for (int i = 0; i < point_num; i++)
        {
            A(i,0) = points[i].x;
            A(i,1) = points[i].y;
            A(i,2) = points[i].z;
        }

        normal_vector = A.colPivHouseholderQr().solve(B);

        for (int j = 0; j < point_num; j++)
        {
            if (fabs(normal_vector(0) * points[j].x + normal_vector(1) * points[j].y + normal_vector(2) * points[j].z + 1.0f) > threhold)
            {
                return false;
            }
        }
        double normal = normal_vector.norm();
        normal_vector.normalize();
        pabcd(0) = normal_vector(0);
        pabcd(1) = normal_vector(1);
        pabcd(2) = normal_vector(2);
        pabcd(3) = 1/normal;

        return true;

    }
    template<typename PointType,typename T>
    PointType transformPoint(PointType point,const Eigen::Quaternion<T> &q,const Eigen::Matrix<T,3,1> &t){
        Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
        ep = q*ep+t;
        point.x = ep.x();
        point.y = ep.y();
        point.z = ep.z();
        return point;
    }

} // namespace IESKFSlam
