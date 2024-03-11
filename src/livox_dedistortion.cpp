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




bool SyncMeasure(MeasureGroup &measgroup) 
{    
  
  if (lidar_buffer.empty() || imu_buffer.empty()) 
  {
      /// Note: this will happen
      return false;
  }

  float sec = imu_buffer.front()->header.stamp.sec;
  float nano = imu_buffer.front()->header.stamp.nanosec;

  float timestamp_imu_front = sec + nano/1000000000; 

  float sec = lidar_buffer.front()->header.stamp.sec;
  float nano = lidar_buffer.front()->header.stamp.nanosec;

  float timestamp_lidar_front = sec + nano/1000000000; 

  float sec = imu_buffer.front()->header.stamp.sec;
  float nano = imu_buffer.front()->header.stamp.nanosec;

  float timestamp_imu_back = sec + nano/1000000000; 


  if (timestamp_imu_front > timestamp_lidar_front) 
  {
      lidar_buffer.clear();
      std::cout << "clear lidar buffer, only happen at the beginning" << std::endl ;
      return false;
  }

  if (timestamp_imu_back < timestamp_lidar_front) 
  {
      return false;
  }
  
  /// Add lidar data, and pop from buffer
  measgroup.lidar = lidar_buffer.front();
  lidar_buffer.pop_front();
  double lidar_time = measgroup.lidar->header.stamp.toSec();

  /// Add imu data, and pop from buffer
  measgroup.imu.clear();
  int imu_cnt = 0;
  for (const auto &imu : imu_buffer) 
  {   
      float sec   = imu->header.stamp.sec;
      float nano  = imu->header.stamp.nanosec;
      float timestamp_imu = sec + nano/1000000000;
      
      double imu_time = timestamp_imu;
      
      if (imu_time <= lidar_time) 
      {
          measgroup.imu.push_back(imu);
          imu_cnt++;
      }
  }
  for (int i = 0; i < imu_cnt; ++i) 
  {
      imu_buffer.pop_front();
  }
  // ROS_DEBUG("add %d imu msg", imu_cnt);

  return true;
}


void ProcessLoop(std::shared_ptr<ImuProcess> p_imu) 
{
  std::cout << "Start ProcessLoop"<< std::endl;
  // 1000 Hz
  rclcpp::Rate(1000);
  
  while (rclcpp::ok()) 
  {
      MeasureGroup meas;
      std::unique_lock<std::mutex> lk(mtx_buffer);
      sig_buffer.wait(lk, [&meas]() -> bool { return SyncMeasure(meas) || b_exit; });
      lk.unlock();

      if (b_exit) 
      {
          std::cout << "b_exit=true, exit" << std::endl;
          break;
      }

      if (b_reset) 
      {
          std::cout << "reset when rosbag play back" << std::endl;
          p_imu->Reset();
          b_reset = false;
          continue;
      }
      p_imu->Process(meas);
      
      //r.sleep();
  }
}


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("repub_node")
    {

      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      // Subscribe to IMU
      sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, default_qos, std::bind(&MinimalSubscriber::imu_callback, this, _1));
      // Subscribe to Pointcloud
      sub_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, default_qos, std::bind(&MinimalSubscriber::pointcloud_callback, this, _1));
      

    }

    float get_timetamps(auto msg)
    {
      // Get timestamp
      float sec = msg->header.stamp.sec;
      float nano = msg->header.stamp.nanosec;

      return sec + nano/1000000000; 
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    { 
            // Get timestamp
      float sec = msg->header.stamp.sec;
      float nano = msg->header.stamp.nanosec;

      float timestamp = sec + nano/1000000000; 

      std::cout  << "get IMU at time:" << timestamp << std::endl;

      //sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg));

      // ROS_DEBUG("get imu at time: %.6f", timestamp);
   
      mtx_buffer.lock();

      if (timestamp < last_timestamp_imu) {
          std::cout <<"imu loop back, clear buffer" << std::endl;
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
      // Get timestamp
      float sec = msg->header.stamp.sec;
      float nano = msg->header.stamp.nanosec;

      float timestamp = sec + nano/1000000000; 
      // Display it
      std::cout  << "get point cloud at time:" << timestamp << std::endl;
      
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
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}