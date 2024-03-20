#include "undistorded-livox-ros2/data_process.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>
#include <iostream>

/// To notify new data
std::mutex mtx_buffer;
std::condition_variable sig_buffer;
bool b_exit = false;
bool b_reset = false;

/// Buffers for measurements
double last_timestamp_lidar = -1;
std::deque<sensor_msgs::msg::PointCloud2::ConstPtr> lidar_buffer;
double last_timestamp_imu = -1;
std::deque<sensor_msgs::msg::Imu::ConstPtr> imu_buffer;


float GetTimeStampROS2(auto msg)
{
  float sec  = msg->header.stamp.sec;
  float nano = msg->header.stamp.nanosec;
  
  return sec + nano/1000000000;
}

void SigHandle(int sig) 
{
    b_exit = true;
    std::cout << "catch sig :" << sig << std::endl;
    sig_buffer.notify_all();
}


bool SyncMeasure(MeasureGroup &measgroup) 
{    
  // IMU buffer empty and Lidar buffer empty  
  if (lidar_buffer.empty() || imu_buffer.empty()) 
  {
      /// Note: this will happen
      return false;
  }
  // Current IMU time > Current Lidar time
  if (GetTimeStampROS2(imu_buffer.front()) > GetTimeStampROS2(lidar_buffer.front())) 
  {
      lidar_buffer.clear();
      //RCLCPP_ERROR("clear lidar buffer, only happen at the beginning");
      return false;
  }
  // Last IMU time < Current Lidar time 
  if (GetTimeStampROS2(imu_buffer.back()) < GetTimeStampROS2(lidar_buffer.front())) 
  {
      return false;
  }


  /// Add lidar data, and pop from buffer
  measgroup.lidar = lidar_buffer.front();
  // Get timestamp from lidar
  double lidar_time = GetTimeStampROS2(measgroup.lidar);
  // remove last lidar in the buffer
  lidar_buffer.pop_front();


  
  // Clear last imu data
  measgroup.imu.clear();
  // Add imu data, and pop from buffer
  int imu_cnt = 0;
  // For all IMU data in the buffer
  for (const auto &imu : imu_buffer) 
  {   
      // Get time
      double imu_time = GetTimeStampROS2(imu);

      if (imu_time <= lidar_time) 
      {   
          // Stack IMU data into vector       
          measgroup.imu.push_back(imu);
          // Increase the counter
          imu_cnt++;
      }
  }
  // For every IMU data saved in the vector
  for (int i = 0; i < imu_cnt; ++i) 
  { 
    // Empty buffer from the saved imu data
    imu_buffer.pop_front();
  }
  
  // RCLCPP_DEBUG("add %d imu msg", imu_cnt);

  return true;
}


void ProcessLoop(std::shared_ptr<ImuProcess> p_imu) 
{
  //RCLCPP_INFO("Start ProcessLoop");
  // Init loop rate at 1000 Hz
  rclcpp::Rate r(1000);
  // If ros not shutdown
  while (rclcpp::ok()) 
  {   
      // Init measurement buffer
      MeasureGroup meas;
      // Init tread (not sure what kind thos)
      std::unique_lock<std::mutex> lk(mtx_buffer);
      // ???
      sig_buffer.wait(lk, [&meas]() -> bool { return SyncMeasure(meas) || b_exit; });
      // Unlock thread
      lk.unlock();

      if (b_exit) 
      {
          //RCLCPP_INFO("b_exit=true, exit");
          break;
      }

      if (b_reset) 
      {
          //RCLCPP_WARN("reset when rosbag play back");
          p_imu->Reset();
          b_reset = false;
          continue;
      }
      p_imu->Process(meas);
      
      r.sleep();
  }
}

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("desdistortion_node")
    {

      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      // Subscribe to IMU
      sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, default_qos, std::bind(&MinimalSubscriber::imu_callback, this, _1));
      // Subscribe to Pointcloud
      sub_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, default_qos, std::bind(&MinimalSubscriber::pointcloud_callback, this, _1));
      

    }


    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    { 
      // Get timestamp
      float timestamp = GetTimeStampROS2(msg); 
      
      RCLCPP_DEBUG(this->get_logger(),"get point cloud at time: %.6f", timestamp);

      //sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg));

      mtx_buffer.lock();

      if (timestamp < last_timestamp_imu) {
          RCLCPP_ERROR(this->get_logger(),"imu loop back, clear buffer");
          imu_buffer.clear();
          b_reset = true;
      }
      last_timestamp_imu = timestamp;

      imu_buffer.push_back(msg);

      mtx_buffer.unlock();
      sig_buffer.notify_all();
      
      
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    { 
      float timestamp = GetTimeStampROS2(msg);
      // Display it
      RCLCPP_DEBUG(this->get_logger(),"get point cloud at time: %.6f", timestamp);

      // Lock the thread
      mtx_buffer.lock();
      if (timestamp < last_timestamp_lidar) 
      {
        std::cout <<"lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
      }
      last_timestamp_lidar = timestamp;
      lidar_buffer.push_back(msg);
      std::cout << "received point size: " << float(msg->data.size())/float(msg->point_step) << "\n";
      // Unlock the tread
      mtx_buffer.unlock();

      sig_buffer.notify_all();
      
    
    }

  
    std::string pointcloud_topic = "/livox/lidar";
    std::string imu_topic        = "/livox/imu";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  sub_pointcloud;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          sub_imu;

    
};

int main(int argc, char * argv[])
{

  std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

  // Init ROS2 node
  rclcpp::init(argc, argv);
  // Init the class
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  signal(SIGINT, SigHandle);
  

  std::vector<double> vec;
  //if(nh.getParam("/ExtIL", vec) )
  {
      Eigen::Quaternion<double> q_il;
      Eigen::Vector3d t_il;
      q_il.w() = vec[0];
      q_il.x() = vec[1];
      q_il.y() = vec[2];
      q_il.z() = vec[3];
      t_il << vec[4], vec[5], vec[6];
      p_imu->set_T_i_l(q_il, t_il);
      
      //RCLCPP_INFO(rclcpp::get_logger(),"Extrinsic Parameter RESET ... ");
  }

  /// for debug
  //p_imu->nh = nh;

  std::thread th_proc(ProcessLoop, p_imu);

  // ros::spin();
  rclcpp::Rate r(1000);

  while (rclcpp::ok()) 
  {
      if (b_exit) break;
      r.sleep();
  }
  
  RCLCPP_INFO(rclcpp->get_logger(),"Wait for process loop exit");
  //if (th_proc.joinable()) th_proc.join();


  rclcpp::shutdown();
  return 0;
}