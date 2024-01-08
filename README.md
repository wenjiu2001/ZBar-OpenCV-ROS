# ZBar-OpenCV-ROS

"zbar_opencv" is a ROS package designed for the straightforward scanning of barcodes.

## Environment

- Ubuntu 20.04
- ROS Noetic

## Requirements

- ZBar
   ```
   sudo apt-get install ros-noetic-zbar-ros*
   ```
- Usbcam
   ```
   sudo apt-get install ros-noetic-usb-cam*
   ```

## Install and Build

1. Navigating to the "src" directory within your catkin workspace :
   ```
   cd ~/catkin_ws/src
   ```
2. Clone zbar_opencv package for github :
   ```
   git clone https://github.com/wenjiu2001/ZBar-OpenCV-ROS.git zbar_opencv
   ```
3. Build zbar_opencv package :
   ```
   cd ~/catkin_ws && catkin_make
   ```
4. Package environment setup :
   ```
   source ./devel/setup.sh
   ```

## How to Use

1. Commence the ROS node for ZBar barcode recognition :
   ```
   rosrun zbar_opencv zbar_opencv
   ```
2. Initiate Usbcam and ZBar barcode recognition :
   ```
   roslaunch zbar_opencv zbar_opencv.launch
   ```
   
## References

- Ubuntu install of ROS Noetic (https://wiki.ros.org/noetic/Installation/Ubuntu)
- zbar_ros (https://wiki.ros.org/zbar_ros)
- usb_cam (https://wiki.ros.org/usb_cam)
