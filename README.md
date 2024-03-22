# livox_cloud_undistortion_ROS2
This project is used for lidar point cloud undistortion. During the recording process, the lidar point cloud has naturally the distortion due to the affect of ego motion. Using the interpolation of ego motion, the distortion can be eliminated by transforming each point to the frame head or tail coordinate. Figures below show an example of undistortion. The left figure shows the row point cloud. The right figure shows the result of rotation distortion deskew. This repo is basically an addaptation of the following code made by Livox on ROS2.

<a href="https://docs.ros.org/en/foxy/Installation.html"><img src="https://img.shields.io/badge/ROS2-foxy-orange" /></a>
<a href="https://docs.ros.org/en/humble/Installation.html"><img src="https://img.shields.io/badge/ROS2-Humble-yellow" /></a>
<a href="https://github.com/Tim-HW/Livox_cloud_undistortion_ROS2/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-green" /></a>




<div align="center">
<img src="./images/before.png" height="300px">
<img src="./images/after.png" height="300px">
</div>

## Prepare the data
Data type: rosbag  
Topics: 
* /livox/lidar   type: sensor_msgs::PointCloud2
* /livox/imu     type: sensor_msgs::Imu

## Dependency
*  ROS2
*  PCL


## Build
In your work space
```
colcon build
```

## Run
```
source install/setup.bash
ros2 run deskew-livox deskew_node
```

## Interface introduction
The member function UndistortPcl of the class ImuProcess defined in the header data_process.h, is used for the point cloud undistortion. The parameter Sophus::SE3d Tbe is the egomotion. If you can provide ego motion, just call this function. Otherwise, the function Process (also a member function of the class ImuProcess) is anothor choice, which uses the IMU data to deskew the rotation distortion.