/*
 * @Descripttion: 矩形地图维护
 * @Author: Meng Kai
 * @version:
 * @Date: 2023-05-26 09:16:11
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-07-05 14:27:45
 */
#pragma once
#include <pcl/common/transforms.h>

#include "libs/ikd-Tree/ikd_Tree.h"
#include "slam_craft/math/geometry.hpp"
#include "slam_craft/modules/eskf/eskf.hpp"
#include "slam_craft/modules/sc_module_base.h"
#include "slam_craft/types/base_type.h"

namespace SlamCraft {
class RectMapManager : private SCModuleBase {
 private:
  double rect_side_length = 1000;
  double map_resolution = 0.5;
//   KDTreePtr kd_tree_ptr;
  std::shared_ptr<KD_TREE<Point>> ikd_tree_ptr;

 public:
  using Ptr = std::shared_ptr<RectMapManager>;
  RectMapManager(const std::string &config_path, const std::string &prefix);
  ~RectMapManager();
  void addScan(PointCloudPtr frame, const Eigen::Quaterniond &att_q, const Eigen::Vector3d &pos_t);
  template <typename Compare>
  bool judgment(const Point &p, const Eigen::Vector3d &t, double thresold,
                const Compare &com) const;
//   PointCloudConstPtr readLocalMap() { return local_map_ptr; }

//   KDTreeConstPtr readKDTree() { return kd_tree_ptr; }
  std::shared_ptr<KD_TREE<Point>> readIKD_Tree() { return ikd_tree_ptr; }
};

RectMapManager::RectMapManager(const std::string &config_path, const std::string &prefix)
    : SCModuleBase(config_path, prefix, "Rectangle Map Manager") {
  ikd_tree_ptr = std::make_shared<KD_TREE<Point>>();
  readParam("map_resolution", map_resolution, 0.5);
  readParam("rect_sid_length", rect_side_length, 1000.0);

//   kd_tree_ptr = pcl::make_shared<KDTree>();
  printParamTable();
}

RectMapManager::~RectMapManager() {}
void RectMapManager::addScan(PointCloudPtr frame_ptr, const Eigen::Quaterniond &att_q,
                             const Eigen::Vector3d &pos_t) {
  PointCloud frame;
  static size_t add_cnt = 0;
  
  if (ikd_tree_ptr->Root_Node == nullptr) {
    pcl::transformPointCloud(*frame_ptr, frame, compositeTransform(att_q, pos_t).cast<float>());
    ikd_tree_ptr->Build(frame.points);
  } else {
    std::vector<Point, Eigen::aligned_allocator<Point>> point_add;
    for (auto &&point : *frame_ptr) {
    //   std::vector<Point, Eigen::aligned_allocator<Point>> pointv;
    //   std::vector<float> distance;
    //   ikd_tree_ptr->Nearest_Search(point, 5, pointv, distance);
      if (point.tag==10 ) {
        Eigen::Vector3d pv = att_q*point.getVector3fMap().cast<double>()+pos_t;
        point.x = pv.x();
        point.y = pv.y();
        point.z = pv.z();
        point_add.push_back(point);
      }
    }
    ikd_tree_ptr->Add_Points(point_add, false);
  }
  //   if (local_map_ptr->empty()) {
  //     *local_map_ptr = frame;
  //     kd_tree_ptr->setInputCloud(local_map_ptr);
  //   } else {
  //     for (auto &&point : frame) {
  //       std::vector<int> ind;
  //       std::vector<float> distance;
  //       kd_tree_ptr->nearestKSearch(point, 5, ind, distance);
  //       if (distance[0] > map_resolution) local_map_ptr->push_back(point);
  //     }
  //     int left = 0, right = local_map_ptr->size() - 1;
  //     while (left < right) {
  //       while (left < right && judgment(local_map_ptr->points[right], pos_t, rect_side_length,
  //                                       std::greater<double>()))
  //         right--;
  //       while (left < right &&
  //              judgment(local_map_ptr->points[left], pos_t, rect_side_length,
  //              std::less<double>()))
  //         left++;
  //       std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);
  //     }
  //     local_map_ptr->resize(right + 1);
  //     kd_tree_ptr->setInputCloud(local_map_ptr);
  //   }
  add_cnt++;
}
template <typename Compare>
bool RectMapManager::judgment(const Point &p, const Eigen::Vector3d &t, double thresold,
                              const Compare &com) const {
  return com(abs(p.x - t.x()), thresold) && com(abs(p.y - t.y()), thresold) &&
         com(abs(p.z - t.z()), thresold);
}

}  // namespace SlamCraft
