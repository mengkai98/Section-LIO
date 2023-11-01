#pragma once
#include "slam_craft/types/base_type.h"
#include "slam_craft/types/imu.h"
#include <deque>

namespace SlamCraft
{
    struct MeasureGroup
    {
        double lidar_begin_time;
        std::deque<IMU> imus;
        PointCloud cloud;
        double lidar_end_time;
    };

} // namespace SlamCraft
