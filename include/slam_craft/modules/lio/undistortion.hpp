#pragma once
#include "slam_craft/modules/eskf/eskf.hpp"
#include "slam_craft/modules/lio/lio_types.hpp"
namespace SlamCraft {
class UndistortionCloud {
 private:
  struct IMUPose6d {
    double time;
    Eigen::Vector3d acc;
    Eigen::Vector3d angvel;
    Eigen::Vector3d vel;
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
    IMUPose6d(double time_ = 0, Eigen::Vector3d a_ = Eigen::Vector3d::Zero(),
              Eigen::Vector3d av_ = Eigen::Vector3d::Zero(), Eigen::Vector3d v_ = Eigen::Vector3d::Zero(),
              Eigen::Vector3d p_ = Eigen::Vector3d::Zero(), Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity()) {
      time = time_;
      acc = a_;
      angvel = av_;
      vel = v_;
      pos = p_;
      rot = q_;
    }
  };

  Eigen::Vector3d acc_s_last;
  Eigen::Vector3d angvel_last;

 public:
  Eigen::Vector3d mean_acc;
  double last_lidar_end_time_;
  IMU last_imu_;
  UndistortionCloud(/* args */);
  ~UndistortionCloud();
  bool operator()(const ESKF::Ptr &eskf_ptr, MeasureGroup &mg);
};

UndistortionCloud::UndistortionCloud(/* args */) {}

UndistortionCloud::~UndistortionCloud() {}
bool UndistortionCloud::operator()(const ESKF::Ptr &eskf_ptr, MeasureGroup &mg) {
  std::sort(mg.cloud.points.begin(), mg.cloud.points.end(),
            [](SlamCraft::Point x, SlamCraft::Point y) { return x.offset_time < y.offset_time; });
  auto &eskf = *eskf_ptr;
  std::vector<IMUPose6d> IMUpose;
  auto v_imu = mg.imus;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front().time_stamp.sec();
  const double &imu_end_time = v_imu.back().time_stamp.sec();
  const double &pcl_beg_time = mg.lidar_begin_time;
  const double &pcl_end_time = mg.lidar_end_time;
  auto &pcl_out = mg.cloud;
  auto imu_state = eskf.getX();
  IMUpose.clear();
  IMUpose.emplace_back(0.0, acc_s_last, angvel_last, imu_state.velocity, imu_state.position, imu_state.rotation);
  Eigen::Vector3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  Eigen::Matrix3d R_imu;
  double dt = 0;
  SlamCraft::IMU in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    if (tail.time_stamp.sec() < last_lidar_end_time_) continue;
    angvel_avr = 0.5 * (head.gyroscope + tail.gyroscope);
    acc_avr = 0.5 * (head.acceleration + tail.acceleration);
    if (mean_acc.norm() > 0.1) {
      acc_avr = acc_avr * GRAVITY / mean_acc.norm();
    }

    if (head.time_stamp.sec() < last_lidar_end_time_) {
      dt = tail.time_stamp.sec() - last_lidar_end_time_;
    } else {
      dt = tail.time_stamp.sec() - head.time_stamp.sec();
    }
    in.acceleration = acc_avr;
    in.gyroscope = angvel_avr;
    eskf.predict(in, dt);
    imu_state = eskf.getX();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rotation * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++) {
      acc_s_last[i] += imu_state.gravity[i];
    }
    double &&offs_t = tail.time_stamp.sec() - pcl_beg_time;
    IMUpose.emplace_back(offs_t, acc_s_last, angvel_last, imu_state.velocity, imu_state.position, imu_state.rotation);
  }
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  eskf.predict(in, dt);
  imu_state = eskf.getX();
  last_imu_ = mg.imus.back();
  last_lidar_end_time_ = pcl_end_time;
  if (pcl_out.points.begin() == pcl_out.points.end()) return false;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu = head->rot.toRotationMatrix();
    vel_imu = head->vel;
    pos_imu = head->pos;
    acc_imu = tail->acc;
    angvel_avr = tail->angvel;
    for (; it_pcl->offset_time / 1e9 > head->time; it_pcl--) {
      dt = it_pcl->offset_time / 1e9 - head->time;
      Eigen::Matrix3d R_i(R_imu * SlamCraft::so3Exp(angvel_avr * dt));
      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.position);
      Eigen::Vector3d P_compensate = imu_state.rotation.conjugate() * (R_i * P_i + T_ei);
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);
      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
  return true;
}
}  // namespace SlamCraft
