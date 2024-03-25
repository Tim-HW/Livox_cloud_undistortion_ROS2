#include "undistorded-livox-ros2/data_process.h"
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

using Sophus::SE3d;
using Sophus::SO3d;


pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudtmp(new pcl::PointCloud<pcl::PointXYZI>());


float ImuProcess::GetTimeStampROS2(auto msg)
{
  float sec  = msg->header.stamp.sec;
  float nano = msg->header.stamp.nanosec;
  
  return sec + nano/1000000000;
}

ImuProcess::ImuProcess() : b_first_frame_(true), last_lidar_(nullptr), last_imu_(nullptr) 
{
    Eigen::Quaterniond q(1, 0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    T_i_l = Sophus::SE3d(q, t);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  RCLCPP_INFO(node_->get_logger(),"Reset ImuProcess");

  b_first_frame_ = true;
  last_lidar_    = nullptr;
  last_imu_      = nullptr;

  gyr_int_.Reset(-1, nullptr);

  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::IntegrateGyr(const std::vector<sensor_msgs::msg::Imu::ConstPtr> &v_imu) 
{
  /// Reset gyr integrator
  gyr_int_.Reset(GetTimeStampROS2(last_lidar_), last_imu_);
  /// And then integrate all the imu measurements
  for (const auto &imu : v_imu) {
    gyr_int_.Integrate(imu);
  }
  RCLCPP_INFO(node_->get_logger(),"integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
           gyr_int_.GetRot().angleX() * 180.0 / M_PI,
           gyr_int_.GetRot().angleY() * 180.0 / M_PI,
           gyr_int_.GetRot().angleZ() * 180.0 / M_PI);
}

void ImuProcess::UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out,
                              double dt_be, const Sophus::SE3d &Tbe) 
{
  const Eigen::Vector3d &tbe = Tbe.translation();
  Eigen::Vector3d rso3_be = Tbe.so3().log();
  for (auto &pt : pcl_in_out->points) 
  {
    int ring = int(pt.intensity);
    float dt_bi = pt.intensity - ring;

    if (dt_bi == 0) laserCloudtmp->push_back(pt);
    double ratio_bi = dt_bi / dt_be;
    /// Rotation from i-e
    double ratio_ie = 1 - ratio_bi;

    Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
    SO3d Rie = SO3d::exp(rso3_ie);

    /// Transform to the 'end' frame, using only the rotation
    /// Note: Compensation direction is INVERSE of Frame's moving direction
    /// So if we want to compensate a point at timestamp-i to the frame-e
    /// P_compensate = R_ei * Pi + t_ei
    Eigen::Vector3d tie = ratio_ie * tbe;
    // Eigen::Vector3d tei = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
    Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

    /// Undistorted point
    pt.x = v_pt_comp_e.x();
    pt.y = v_pt_comp_e.y();
    pt.z = v_pt_comp_e.z();
  }
}

std::vector<sensor_msgs::msg::PointCloud2> ImuProcess::Process(const MeasureGroup &meas) 
{
  //RCLCPP_ASSERT(!meas.imu.empty());
  //RCLCPP_ASSERT(meas.lidar != nullptr);

  std::vector<sensor_msgs::msg::PointCloud2> PclToPublish;
  
  RCLCPP_INFO(node_->get_logger(),"Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
            GetTimeStampROS2(meas.lidar), meas.imu.size(),
            GetTimeStampROS2(meas.imu.front()),
            GetTimeStampROS2(meas.imu.back()));

  auto pcl_in_msg = meas.lidar;
  
  if (b_first_frame_) 
  {
    /// The very first lidar frame

    /// Reset
    Reset();

    /// Record first lidar, and first useful imu
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();

    RCLCPP_WARN(node_->get_logger(),"The very first lidar frame");
    /// Do nothing more, return
    b_first_frame_ = false;
    // Return an empty vector of pointcloud
    return PclToPublish;
  }

  /// Integrate all input imu message
  IntegrateGyr(meas.imu);

  /// Compensate lidar points with IMU rotation         
  /// Initial pose from IMU (with only rotation)
  SE3d T_l_c(gyr_int_.GetRot(), Eigen::Vector3d::Zero());
  dt_l_c_ = GetTimeStampROS2(pcl_in_msg) - GetTimeStampROS2(last_lidar_);
  //// Get input pcl
  pcl::fromROSMsg(*pcl_in_msg, *cur_pcl_in_);

  /// Undistort points

  Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;
  pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);
  clock_t t1,t2;
  t1 = clock();
  UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);
  t2 = clock();
  //printf("time is: %f\n", 1000.0*(t2 - t1) / CLOCKS_PER_SEC);


  {
    // First Pointcloud
    sensor_msgs::msg::PointCloud2 pcl_out_msg; 
    pcl::toROSMsg(*laserCloudtmp, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "/livox_frame";
    PclToPublish.push_back(pcl_out_msg);
    laserCloudtmp->clear();
  }

  { 
    // Undistorded
    sensor_msgs::msg::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "/livox_frame";
    PclToPublish.push_back(pcl_out_msg);
  }
  { 
    // Origin
    sensor_msgs::msg::PointCloud2 pcl_out_msg;
    //std::cout << "point size: " << cur_pcl_in_->points.size() << "\n";
    pcl::toROSMsg(*cur_pcl_in_, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "/livox_frame";
    PclToPublish.push_back(pcl_out_msg);
  }

  /// Record last measurements
  last_lidar_ = pcl_in_msg;
  last_imu_   = meas.imu.back();
  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());

  return PclToPublish; 
}