/*
 * @Descripttion: 
 * @Author: Meng Kai
 * @version: 
 * @Date: 2023-05-23 21:14:47
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-05-30 20:30:04
 */
#pragma once
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <pcl/impl/pcl_base.hpp>
#include <pcl/pcl_base.h>
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/point_types.h>
#include <color_terminal/color_terminal.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_craft/types/time_stamp.h"
namespace SlamCraft {


    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        std::uint32_t offset_time;
        std::int32_t ring;
        std::uint8_t tag;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    class PointCloud: public pcl::PointCloud<Point>
    {
    public:
        TimeStamp time_stamp;
    };
    using VoxelFilter = pcl::VoxelGrid<Point>;
    using PointCloudPtr = PointCloud::Ptr;
    using PointCloudConstPtr = PointCloud::ConstPtr;
    using KDTree = pcl::KdTreeFLANN<Point>;
    using KDTreePtr = KDTree::Ptr;
    using KDTreeConstPtr = KDTree::ConstPtr;

    template<typename _first, typename _second, typename _thrid>
    struct triple{
        _first first;
        _second second;
        _thrid thrid;
    };
    const double GRAVITY = 9.81;
} 
POINT_CLOUD_REGISTER_POINT_STRUCT(SlamCraft::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::int32_t, ring, ring)
    (std::uint8_t, tag, tag)
)

#define LIST_EXPAND_3(VAR)   {VAR.x,VAR.y,VAR.z};
#define VECTOR_EXPAND_3(VAR) VAR[0],VAR[1],VAR[2]
#define VECTOR_EXPAND_9(VAR) VAR[0],VAR[1],VAR[2],VAR[3],VAR[4],VAR[5],VAR[6],VAR[7],VAR[8]