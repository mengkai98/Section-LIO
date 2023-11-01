/*
 * @Descripttion: 
 * @Author: Meng Kai
 * @version: 
 * @Date: 2023-05-26 00:00:44
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-05-30 00:22:10
 */
#pragma once
#include "slam_craft/types/base_type.h"
namespace SlamCraft
{
    
    template<typename PointType,typename T>
    PointType transformPoint(PointType point,const Eigen::Quaternion<T> &q,const Eigen::Matrix<T,3,1> &t){
        Eigen::Matrix<T,3,1> ep = LIST_EXPAND_3(point);
        ep = q*ep+t;
        point.x = ep.x();
        point.y = ep.y();
        point.z = ep.z();
        return point;
    }

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

    Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond & q,const Eigen::Vector3d & t){
        Eigen::Matrix4d ans;
        ans.setIdentity();
        ans.block<3,3>(0,0) = q.toRotationMatrix();
        ans.block<3,1>(0,3) = t;
        return ans;
    }
} // namespace SlamCraft
