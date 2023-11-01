#pragma once
#include <fstream>

#include "craft_extension/time/tictoc.hpp"
#include "slam_craft/modules/lio/lio_zh_model.hpp"
#include "slam_craft/modules/lio/undistortion.hpp"
#include "slam_craft/modules/map_manager/rect_map_manager.hpp"
#include "slam_craft/modules/section_process/section_process.hpp"
#include "slam_craft/utils/tic_toc.hpp"
namespace SlamCraft {
class SectionalLIO : private SCModuleBase {
 public:
  TimeStamp nowtime;
  SectionalLIO(std::string config_file_path, std::string prefix);
  ~SectionalLIO() {  }
  bool process();
  void addImu(const IMU &imu);
  void addCloud(PointCloud cloud);
  ESKF::State18 readState() { return eskf_ptr->getX(); }
  PointCloudConstPtr readNowPointCloud() { return filter_point_cloud_ptr; }

  Eigen::Quaterniond readExtrinRotation() { return extrin_r; }
  Eigen::Vector3d readExtrinPosition() { return extrin_t; }

 private:
  std::deque<IMU> imu_buffer;
  std::deque<PointCloud> cloud_buffer;
  ESKF::Ptr eskf_ptr;
  VoxelFilter::Ptr voxel_filter_ptr;
  Eigen::Vector3d mean_acc;
  IMU last_imu_;

  Eigen::Quaterniond extrin_r;
  Eigen::Vector3d extrin_t;

  UndistortionCloud undistortion_cloud;
  LIOZHModel::Ptr lio_zh_model_ptr;
  SectionProcess::Ptr section_process_ptr;

  PointCloudPtr filter_point_cloud_ptr;
  RectMapManager::Ptr rmm_ptr;
  KDTreeConstPtr kd_tree_ptr;
  bool record_mode = false;
  int section_min_map_size = 0;

  bool imu_inited;
  bool syncMeasureGroup(MeasureGroup &mg);
  void init_state(MeasureGroup &mg);
};
SectionalLIO::SectionalLIO(std::string config_file_path, std::string prefix)
    : SCModuleBase(config_file_path, prefix, "Sectional LIO") {
  imu_inited = false;

  voxel_filter_ptr = pcl::make_shared<VoxelFilter>();
  float leaf_size;
  readParam("filter_leaf_size", leaf_size, 0.5f);
  voxel_filter_ptr->setLeafSize(leaf_size, leaf_size, leaf_size);

  std::vector<double> extrin_v;
  readParam("extrin_r", extrin_v, std::vector<double>());
  extrin_r.setIdentity();
  extrin_t.setZero();
  if (extrin_v.size() == 9) {
    Eigen::Matrix3d extrin_r33;
    extrin_r33 << VECTOR_EXPAND_9(extrin_v);
    extrin_r = extrin_r33;
  } else if (extrin_v.size() == 3) {
    extrin_r.x() = extrin_v[0];
    extrin_r.y() = extrin_v[1];
    extrin_r.z() = extrin_v[2];
    extrin_r.w() = extrin_v[3];
  }
  readParam("extrin_t", extrin_v, std::vector<double>());
  if (extrin_v.size() == 3) {
    extrin_t << VECTOR_EXPAND_3(extrin_v);
  }

  eskf_ptr = std::make_shared<ESKF>(config_file_path, "eskf");
  lio_zh_model_ptr = std::make_shared<LIOZHModel>();
  eskf_ptr->calc_zh_ptr = lio_zh_model_ptr;

  rmm_ptr = std::make_shared<RectMapManager>(config_file_path, "rect_map_manager");

  section_process_ptr = std::make_shared<SectionProcess>(config_file_path, "section_process");

  filter_point_cloud_ptr = pcl::make_shared<PointCloud>();
  lio_zh_model_ptr->prepare(rmm_ptr->readIKD_Tree(), filter_point_cloud_ptr);
  section_process_ptr->prepare(rmm_ptr->readIKD_Tree());
  readParam("section_min_map_size", section_min_map_size, 10000);

  printParamTable();
}
void SectionalLIO::addImu(const IMU &imu) { imu_buffer.push_back(imu); }
void SectionalLIO::addCloud(PointCloud cloud) {
  for (auto &&point : cloud) {
    Eigen::Vector3d point_eigen = {point.x, point.y, point.z};
    point_eigen = extrin_r * point_eigen + extrin_t;
    point.x = point_eigen.x();
    point.y = point_eigen.y();
    point.z = point_eigen.z();

    point.tag = 0;
  }

  cloud_buffer.push_back(cloud);
}
void SectionalLIO::init_state(MeasureGroup &mg) {
  static int imu_count = 0;
  auto &eskf = *eskf_ptr;
  if (imu_inited) {
    return;
  }

  for (size_t i = 0; i < mg.imus.size(); i++) {
    imu_count++;
    auto x = eskf.getX();
    x.gravity += mg.imus[i].acceleration;
    x.bg += mg.imus[i].gyroscope;
    eskf.setX(x);
  }
  if (imu_count >= 5) {
    auto x = eskf.getX();
    x.gravity /= double(imu_count);

    x.bg /= double(imu_count);
    mean_acc = x.gravity;
    undistortion_cloud.mean_acc = mean_acc;
    if (mean_acc.norm() < 0.1) {
      x.gravity.setZero();
    } else {
      x.gravity = -mean_acc / mean_acc.norm() * SlamCraft::GRAVITY;
    }
    eskf.setX(x);
    imu_inited = true;
    undistortion_cloud.mean_acc = mean_acc;
  }
  last_imu_ = mg.imus.back();
  undistortion_cloud.last_imu_ = last_imu_;
  undistortion_cloud.last_lidar_end_time_ = mg.lidar_end_time;
  return;
}
bool SectionalLIO::process() {
  MeasureGroup mg;
  static int converge_cnt = 0;
  static int process_times = 0;
  std::vector<float> times;
  if (syncMeasureGroup(mg)) {
    nowtime = mg.cloud.time_stamp;
    if (!imu_inited) {
      init_state(mg);
      return false;
    }
    bool converge = false;
    if (rmm_ptr->readIKD_Tree()->size() < section_min_map_size) {
      undistortion_cloud(eskf_ptr, mg);
      voxel_filter_ptr->setInputCloud(mg.cloud.makeShared());
      voxel_filter_ptr->filter(*filter_point_cloud_ptr);

      if (rmm_ptr->readIKD_Tree()->size()==0) {
        auto x = eskf_ptr->getX();
        rmm_ptr->addScan(filter_point_cloud_ptr, x.rotation, x.position);
        return false;
      }

      converge = eskf_ptr->update();
      auto x = eskf_ptr->getX();
      if (converge) {
        rmm_ptr->addScan(filter_point_cloud_ptr, x.rotation, x.position);
      }

    } else {
      TicToc tt;
      std::vector<MeasureGroup> sections;
      auto x = eskf_ptr->getX();
      tt.tic();
      section_process_ptr->process(mg, sections, x.rotation, x.position);
      times.push_back(tt.toc());//0
      PointCloud uncloud;
      int cnt = 0;
      tt.tic();
      for (auto &&section : sections) {
        undistortion_cloud(eskf_ptr, section);
        voxel_filter_ptr->setInputCloud(section.cloud.makeShared());
        voxel_filter_ptr->filter(*filter_point_cloud_ptr);
        converge = eskf_ptr->update();
        x = eskf_ptr->getX();
        pcl::transformPointCloud(*filter_point_cloud_ptr, section.cloud,
                                 compositeTransform(x.rotation, x.position).cast<float>());
        for (auto &&section_point : section.cloud) {
          section_point.tag = cnt;
        }
        cnt++;
        uncloud += section.cloud;
      }
      x = eskf_ptr->getX();
      tt.tic();
      for (auto &&point : uncloud) {
        Eigen::Vector3d point_eigen = {point.x, point.y, point.z};
        point_eigen = x.rotation.conjugate() * (point_eigen - x.position);
        point.x = point_eigen.x();
        point.y = point_eigen.y();
        point.z = point_eigen.z();
      }

      *filter_point_cloud_ptr = uncloud;

      converge = eskf_ptr->update();

      x = eskf_ptr->getX();
      if (converge) {
        rmm_ptr->addScan(filter_point_cloud_ptr, x.rotation, x.position);
      }
    }
    process_times++;
    auto state = eskf_ptr->getX();
    return true;
  }
  return false;
}
bool SectionalLIO::syncMeasureGroup(MeasureGroup &mg) {
  mg.imus.clear();
  mg.cloud.clear();
  if (cloud_buffer.empty() || imu_buffer.empty()) {
    return false;
  }
  double imu_end_time = imu_buffer.back().time_stamp.sec();
  double imu_start_time = imu_buffer.front().time_stamp.sec();
  double cloud_start_time = cloud_buffer.front().time_stamp.sec();
  double cloud_end_time = cloud_buffer.front().points.back().offset_time / 1e9 + cloud_start_time;

  if (imu_end_time < cloud_end_time) {
    return false;
  }

  if (cloud_end_time < imu_start_time) {
    cloud_buffer.pop_front();
    return false;
  }
  mg.cloud = cloud_buffer.front();
  cloud_buffer.pop_front();
  mg.lidar_begin_time = cloud_start_time;
  mg.lidar_end_time = cloud_end_time;
  while (!imu_buffer.empty()) {
    if (imu_buffer.front().time_stamp.sec() < mg.lidar_end_time) {
      mg.imus.push_back(imu_buffer.front());
      imu_buffer.pop_front();

    } else {
      break;
    }
  }
  if (mg.imus.size() <= 5) {
    return false;
  }
  return true;
}
}  // namespace SlamCraft
