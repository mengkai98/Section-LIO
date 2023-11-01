#pragma once
#include "slam_craft/defines.h"
#include "ros/ros.h"
#include "slam_craft/modules/lio/sectional_lio.hpp"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "livox_ros_driver/CustomMsg.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <pcl_conversions/pcl_conversions.h>
#include "wrapper/ros_noetic/lidar_process.h"
using std::string;
enum LidarType{
    AVIA = 0,
    RS16 = 1,
    MULRAN = 2,
    KITTI = 3,
    FORD  = 4,
    VELO = 5,
};
class SectionalLioNoeticWrapper
{
private:
    string lidar_topic_sub;
    string imu_topic_sub;
    int lidar_type;

    ros::Subscriber lidar_sub;
    ros::Subscriber imu_sub;
    ros::Publisher current_cloud_pub;
    ros::Publisher current_odom_pub;
    ros::Publisher current_path_pub;
    ros::Publisher local_map_pub;
    std::shared_ptr<SlamCraft::SectionalLIO>slio_ptr;
    std::shared_ptr<CommonLidarProcessInterface>  lidar_preprocess_ptr;
public:
    SectionalLioNoeticWrapper(ros::NodeHandle&nh);
    ~SectionalLioNoeticWrapper();
    void imuCallback(const sensor_msgs::Imu::Ptr &msg);
    void lidarCallback(const sensor_msgs::PointCloud2::Ptr &msg);
    void livoxCallback(const livox_ros_driver::CustomMsgPtr &msg);
    void run();
    void publishMsg();
};

SectionalLioNoeticWrapper::SectionalLioNoeticWrapper(ros::NodeHandle&nh)
{

  
    string config_file;
    nh.param<string>("wrapper/config_file",config_file,"");
    slio_ptr = std::make_shared<SlamCraft::SectionalLIO>(SLAM_CRAFT_CONFIG_DIR+config_file,"sectional_lio");

    nh.param<string>("wrapper/lidar_topic",lidar_topic_sub,"/lidar");
    nh.param<string>("wrapper/imu_topic",imu_topic_sub,"/imu");
    nh.param<int>("wrapper/lidar_type",lidar_type,AVIA);

    imu_sub = nh.subscribe(imu_topic_sub,100000,&SectionalLioNoeticWrapper::imuCallback,this);
    if(lidar_type==0){
        lidar_sub = nh.subscribe(lidar_topic_sub,10000,&SectionalLioNoeticWrapper::livoxCallback,this);
    }else{
        lidar_sub = nh.subscribe(lidar_topic_sub,10000,&SectionalLioNoeticWrapper::lidarCallback,this);

        if(lidar_type == VELO){
            lidar_preprocess_ptr = std::make_shared<VelodyneProcess>();
        }
        else{
            std::cout<<"unsupport lidar"<<std::endl;
            exit(0);
        }
    }

    current_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/slio/current_cloud",10000);
    current_odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry",10000);
    current_path_pub = nh.advertise<nav_msgs::Path>("/slio/path",10000);
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/slio/local_map",1000);
    run();
}
void SectionalLioNoeticWrapper::imuCallback(const sensor_msgs::Imu::Ptr &msg){

    SlamCraft::IMU imu;
    imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
    imu.acceleration = LIST_EXPAND_3(msg->linear_acceleration);
    imu.gyroscope = LIST_EXPAND_3(msg->angular_velocity);
    slio_ptr->addImu(imu);
    
}
void SectionalLioNoeticWrapper::lidarCallback(const sensor_msgs::PointCloud2::Ptr &msg){

    SlamCraft::PointCloud cloud;

    lidar_preprocess_ptr->process(*msg,cloud);

    slio_ptr->addCloud(cloud);
}
void SectionalLioNoeticWrapper::livoxCallback(const livox_ros_driver::CustomMsgPtr &msg){
    SlamCraft::PointCloud cloud;
    cloud.clear();
    cloud.time_stamp.fromNsec(msg->header.stamp.toNSec());
    for (size_t i = 0; i < msg->points.size(); i++)
    {
        SlamCraft::Point point;
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        point.z = msg->points[i].z;
        point.intensity = msg->points[i].reflectivity;
        point.offset_time = msg->points[i].offset_time;
        point.ring = msg->points[i].line;
        cloud.push_back(point);
    }
    slio_ptr->addCloud(cloud);
}
void SectionalLioNoeticWrapper::run(){
    ros::Rate rate(200);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
        if(slio_ptr->process()){
            publishMsg();
        }
    }
}
void SectionalLioNoeticWrapper::publishMsg(){
    static nav_msgs::Path path;
    

    auto x = slio_ptr->readState();
    sensor_msgs::PointCloud2 ros_local_map,ros_current_cloud;

    SlamCraft::PointCloud cloud;
    pcl::transformPointCloud(*slio_ptr->readNowPointCloud(),cloud,SlamCraft::compositeTransform(slio_ptr->readExtrinRotation(),slio_ptr->readExtrinPosition()).cast<float>());
    pcl::transformPointCloud(cloud,cloud,SlamCraft::compositeTransform(x.rotation,x.position).cast<float>());
    pcl::toROSMsg(cloud,ros_current_cloud);
    // pcl::toROSMsg(*slio_ptr->readLocalMap(),ros_local_map);
    ros_current_cloud.header.frame_id = "map";
    ros_current_cloud.header.stamp = ros::Time().fromNSec(slio_ptr->nowtime.nsec());
    ros_local_map.header = ros_current_cloud.header;
    current_cloud_pub.publish(ros_current_cloud);
    // local_map_pub.publish(ros_local_map);
    nav_msgs::Odometry odom;

    odom.pose.pose.position.x = x.position.x();
    odom.pose.pose.position.y = x.position.y();
    odom.pose.pose.position.z = x.position.z();
    odom.pose.pose.orientation.w = x.rotation.w();
    odom.pose.pose.orientation.x = x.rotation.x();
    odom.pose.pose.orientation.y = x.rotation.y();
    odom.pose.pose.orientation.z = x.rotation.z();
    odom.header = ros_current_cloud.header;
    current_odom_pub.publish(odom);
    
    path.header = ros_current_cloud.header;
    geometry_msgs::PoseStamped psd;
    psd.pose = odom.pose.pose;
    path.poses.push_back(psd);
    current_path_pub.publish(path);
}
SectionalLioNoeticWrapper::~SectionalLioNoeticWrapper()
{
}
