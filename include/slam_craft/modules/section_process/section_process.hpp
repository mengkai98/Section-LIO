/*
 * @Descripttion:
 * @Author: Meng Kai
 * @version:
 * @Date: 2023-05-30 19:15:35
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-07-04 19:18:51
 */
#pragma once
#include "glog/logging.h"
#include "slam_craft/math/geometry.hpp"
#include "slam_craft/modules/lio/lio_types.hpp"
#include "slam_craft/modules/sc_module_base.h"
#include "slam_craft/utils/tic_toc.hpp"
namespace SlamCraft {
class SectionProcess : private SCModuleBase {
 private:
  struct FeatureAndLine {
    int line = 0;
    Eigen::Vector3d feat;
    bool vailed = false;
  };
  const int NEAR_POINTS_NUM = 5;
  int max_sections = 0;
  int section_min_point_size;
  float planaer_threhold;
  float feature_normal_thre;
  bool simple_mode;
  KDTreeConstPtr local_map_kd_tree_ptr;

  PointCloudConstPtr local_map_ptr;
  std::shared_ptr<KD_TREE<Point>> ikd_tree_ptr;

 public:
  using Ptr = std::shared_ptr<SectionProcess>;
  SectionProcess(const std::string &config_file, const std::string &prefix);
  ~SectionProcess();
  void prepare(KDTreeConstPtr kd_tree_ptr, PointCloudConstPtr cloud_ptr);

  bool process(MeasureGroup &mg, std::vector<MeasureGroup> &sections, Eigen::Quaterniond predict_q,
               Eigen::Vector3d predict_t);
  bool process(MeasureGroup &mg, std::vector<MeasureGroup> &sections);
  void prepare(std::shared_ptr<KD_TREE<Point>> kd_tree_ptr);
};

void SectionProcess::prepare(KDTreeConstPtr kd_tree_ptr, PointCloudConstPtr cloud_ptr) {
  local_map_kd_tree_ptr = kd_tree_ptr;
  local_map_ptr = cloud_ptr;
}

void SectionProcess::prepare(std::shared_ptr<KD_TREE<Point>> kd_tree_ptr) {
  ikd_tree_ptr = kd_tree_ptr;
}

bool SectionProcess::process(MeasureGroup &mg, std::vector<MeasureGroup> &sections,
                             Eigen::Quaterniond predict_q, Eigen::Vector3d predict_t) {
    sections.push_back(mg);

  return true;
}

SectionProcess::SectionProcess(const std::string &config_file, const std::string &prefix)
    : SCModuleBase(config_file, prefix, "Section Process") {
  readParam("max_sections", max_sections, 4);
  readParam("planaer_threhold", planaer_threhold, 0.2f);
  readParam("section_min_point_size", section_min_point_size, 4000);
  readParam("feature_normal_thre", feature_normal_thre, 0.f);
  readParam("simple_mode", simple_mode, false);
  feature_normal_thre = sin(feature_normal_thre * M_PI / 180.0);
  printParamTable();
}

SectionProcess::~SectionProcess() {}

}  // namespace SlamCraft
