# ZBar-OpenCV-ROS

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC_BY--NC--SA_4.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Ubuntu:Focal](https://img.shields.io/badge/Ubuntu-Focal-brightgreen)](https://releases.ubuntu.com/focal/)
[![ROS:Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](https://wiki.ros.org/noetic/Installation/Ubuntu)

"zbar_opencv" is a ROS package designed for the straightforward scanning of barcodes.

## Requirements

- ZBar
   ```
   sudo apt-get install libzbar-dev && pip install zbar
   ```
- UsbCam
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

- Initiate usbcam and ZBar barcode recognition :
   ```
   roslaunch zbar_opencv zbar_opencv.launch
   ```
   
## References

- Ubuntu install of ROS Noetic (https://wiki.ros.org/noetic/Installation/Ubuntu)
- zbar_ros (https://wiki.ros.org/zbar_ros)
- usb_cam (https://wiki.ros.org/usb_cam)
