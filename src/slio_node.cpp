/*
 * @Descripttion: 
 * @Author: Meng Kai
 * @version: 
 * @Date: 2023-05-24 22:05:10
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-05-31 15:09:22
 */
#include "ros/ros.h"
#include "wrapper/ros_noetic/sectional_lio_wrapper.hpp"
#include <glog/logging.h>
int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"sectional_lio_node");
    FLAGS_log_dir = SLAM_CRAFT_DIR+"/log";
    google::InitGoogleLogging(argv[0]);
    ros::NodeHandle nh;
    auto slio_noetic_wrapper_ptr = std::make_shared<SectionalLioNoeticWrapper>(nh);
    return 0;
}
