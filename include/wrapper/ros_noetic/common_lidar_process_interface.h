/*
 * @Descripttion: Ros PointCloud2
 * @Author: Meng Kai
 * @version: 
 * @Date: 2023-05-30 17:06:51
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-06-07 18:59:08
 */
#pragma once
#include "slam_craft/types/base_type.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
class CommonLidarProcessInterface
{
public:
    virtual bool process(const sensor_msgs::PointCloud2 &msg,SlamCraft::PointCloud&cloud) = 0;
};

