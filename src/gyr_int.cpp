#include "undistorded-livox-ros2/gyr_int.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

using Sophus::SO3d;


float GetTimeStampROS2(auto msg)
{
  float sec  = msg->header.stamp.sec;
  float nano = msg->header.stamp.nanosec;
  
  return sec + nano/1000000000;
}

GyrInt::GyrInt() : start_timestamp_(-1), last_imu_(nullptr) {}

void GyrInt::Reset(double start_timestamp,
                   const sensor_msgs::msg::Imu::ConstPtr &lastimu) {
  start_timestamp_ = start_timestamp;
  last_imu_ = lastimu;

  v_rot_.clear();
  v_imu_.clear();
}

const Sophus::SO3d GyrInt::GetRot() const {
  if (v_rot_.empty()) {
    return SO3d();
  } else {
    return v_rot_.back();
  }
}

void GyrInt::Integrate(const sensor_msgs::msg::Imu::ConstPtr &imu) {
  /// Init
  if (v_rot_.empty()) {
    //std::cout << start_timestamp_ > 0 << std::endl;
    //std::cout << last_imu_ != nullptr << std::endl;

    /// Identity rotation
    v_rot_.push_back(SO3d());

    /// Interpolate imu in
    sensor_msgs::msg::Imu::Ptr imu_inter(new sensor_msgs::msg::Imu());
    double dt1 = start_timestamp_ - GetTimeStampROS2(last_imu_);
    double dt2 = GetTimeStampROS2(imu) - start_timestamp_;
    //ROS_ASSERT_MSG(dt1 >= 0 && dt2 >= 0, "%f - %f - %f",
    //               GetTimeStampROS2(last_imu_), start_timestamp_,
    //               GetTimeStampROS2(imu));
    double w1 = dt2 / (dt1 + dt2 + 1e-9);
    double w2 = dt1 / (dt1 + dt2 + 1e-9);

    const auto &gyr1 = last_imu_->angular_velocity;
    const auto &acc1 = last_imu_->linear_acceleration;
    const auto &gyr2 = imu->angular_velocity;
    const auto &acc2 = imu->linear_acceleration;

    imu_inter->header.stamp.set__sec(start_timestamp_);
    //imu_inter->header.stamp.fromSec(start_timestamp_);
    imu_inter->angular_velocity.x = w1 * gyr1.x + w2 * gyr2.x;
    imu_inter->angular_velocity.y = w1 * gyr1.y + w2 * gyr2.y;
    imu_inter->angular_velocity.z = w1 * gyr1.z + w2 * gyr2.z;
    imu_inter->linear_acceleration.x = w1 * acc1.x + w2 * acc2.x;
    imu_inter->linear_acceleration.y = w1 * acc1.y + w2 * acc2.y;
    imu_inter->linear_acceleration.z = w1 * acc1.z + w2 * acc2.z;

    v_imu_.push_back(imu_inter);
  }

  ///
  const SO3d &rot_last = v_rot_.back();
  const auto &imumsg_last = v_imu_.back();
  const double &time_last = GetTimeStampROS2(imumsg_last);
  Eigen::Vector3d gyr_last(imumsg_last->angular_velocity.x,
                           imumsg_last->angular_velocity.y,
                           imumsg_last->angular_velocity.z);
  double time = GetTimeStampROS2(imu);
  Eigen::Vector3d gyr(imu->angular_velocity.x, imu->angular_velocity.y,
                      imu->angular_velocity.z);
  assert(time >= 0);
  double dt = time - time_last;
  auto delta_angle = dt * 0.5 * (gyr + gyr_last);
  auto delta_r = SO3d::exp(delta_angle);

  SO3d rot = rot_last * delta_r;

  v_imu_.push_back(imu);
  v_rot_.push_back(rot);
}