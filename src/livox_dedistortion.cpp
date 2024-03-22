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
#include <chrono>
#include <functional>


std::shared_ptr<rclcpp::Node> node;
std::string pointcloud_topic = "/livox/lidar";
std::string imu_topic        = "/livox/imu";
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  sub_pointcloud;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          sub_imu;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pub_UndistortPcl;




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

    std::cout << "Wait for process loop exit" << std::endl;
    
  
  rclcpp::shutdown();
}




bool SyncMeasure(MeasureGroup &measgroup) 
{    
  // IMU buffer empty and Lidar buffer empty  
  if (lidar_buffer.empty() || imu_buffer.empty()) 
  {
      /// Note: this will happen
      std::cout << "buffers empty " << std::endl;
      return false;
  }
  // Current IMU time > Current Lidar time
  if (GetTimeStampROS2(imu_buffer.front()) > GetTimeStampROS2(lidar_buffer.front())) 
  {
      lidar_buffer.clear();
      std::cout <<"clear lidar buffer, only happen at the beginning"<< std::endl;
      return false;
  }
  // Last IMU time < Current Lidar time 
  if (GetTimeStampROS2(imu_buffer.back()) < GetTimeStampROS2(lidar_buffer.front())) 
  {
      return false;
  }

  std::cout<< "add data to buffers" << std::endl;

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
  
  std::cout <<"add "<< imu_cnt <<"imu msg";

  return true;
}

void ProcessLoop(std::shared_ptr<ImuProcess> p_imu) 
{
  std::cout << "Start ProcessLoop" << std::endl;

  rclcpp::Rate r(1000);
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
      r.sleep();
  }

}

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */



void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{ 
  // Get timestamp
  float timestamp = GetTimeStampROS2(msg); 
    
  std::cout<<"get IMU at time: "<< timestamp<< std::endl;
  // Lock the variable
  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) 
  {
      std::cout <<"imu loop back, clear buffer" << std::endl;
      imu_buffer.clear();
      b_reset = true;
  }
  last_timestamp_imu = timestamp;
  // Add current IMU to the buffer
  imu_buffer.push_back(msg);
  // Unlock variable
  mtx_buffer.unlock();
  // Notify
  sig_buffer.notify_all();
  
  
}

void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{ 
  float timestamp = GetTimeStampROS2(msg);
  // Display it
  std::cout <<" get point cloud at time: "<< timestamp << std::endl;

  // Lock the thread
  mtx_buffer.lock();
  if (timestamp < last_timestamp_lidar) 
  {
    std::cout <<"lidar loop back, clear buffer" << std::endl;
    lidar_buffer.clear();
  }
  last_timestamp_lidar = timestamp;
  lidar_buffer.push_back(msg);
  //std::cout << "received point size: " << float(msg->data.size())/float(msg->point_step) << "\n";
  // Unlock the tread
  mtx_buffer.unlock();

  sig_buffer.notify_all();
  

}



int main(int argc, char * argv[])
{
  // Create signal handler
  signal(SIGINT, SigHandle);
  // Init rclcpp
  rclcpp::init(argc, argv);
  // Create node
  node = rclcpp::Node::make_shared("deskew_node");
  // 
  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  // Subscribe to IMU
  sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, default_qos, imu_callback);
  // Subscribe to Pointcloud
  sub_pointcloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, default_qos, pointcloud_callback);
      
  pub_UndistortPcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_first_point", 100);

  // Init Imu process object
  std::shared_ptr<ImuProcess> p_imu(new ImuProcess());
  // Create thread for sync and intergration
  std::thread th_proc(ProcessLoop, p_imu);

  // Init ROS2 spin rate
  rclcpp::Rate r(1000);
  // While ROS2 running 
  while (rclcpp::ok()) 
  {   
      // Exit if ctrl+c
      if (b_exit) break;
      // spin the node
      rclcpp::spin(node);
      // Wait for 0.1ms
      r.sleep();
  }
  // Exit the program
  RCLCPP_INFO(node->get_logger(),"Wait for process loop exit");
  if (th_proc.joinable()) th_proc.join();
  // exit
  return 0;
}