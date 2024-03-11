#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>


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
      sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/livox/imu", default_qos, std::bind(&MinimalSubscriber::imu_callback, this, _1));
      // Subscribe to Pointcloud
      sub_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", default_qos, std::bind(&MinimalSubscriber::pointcloud_callback, this, _1));
      
      std::mutex mtx_buffer;
    }

  

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    { 
      float timestamp = msg->header.stamp.sec;

      std::cout  << "get IMU at time:" << timestamp << std::endl;

      //sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg));

      // ROS_DEBUG("get imu at time: %.6f", timestamp);
      /*
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
      */
      
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    { 
      // Get timestamp
      float sec = msg->header.stamp.sec;
      float nano = msg->header.stamp.nanosec;

      float timestamp = sec + nano/1000000000; 
      // Display it
      std::cout  << "get point cloud at time:" << timestamp << std::endl;
      /*
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
      */
    
    }
  

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  sub_pointcloud;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr  sub_imu;

   
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}