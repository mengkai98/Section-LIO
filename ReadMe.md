# Section-LIO: A High Accuracy LiDAR-Inertial Odometry Using Undistorted Sectional Point

## 1 Prerequisites

Before using Section-LIO, make sure you have the following prerequisites:

- Ubuntu 20.04
- ROS == Noetic
- PCL
- Eigen
- Livox_ros_driver: Follow the [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver) guide to install.

## 2 Build

To build Section-LIO, follow these steps:

```bash
# Create catkin_ws
mkdir -p catkin_ws/src
cd catkin_ws/src

# Clone the code
git clone https://github.com/mengkai98/Section-LIO.git

# Use catkin_make to build the package
cd ..
catkin_make
```

## 3 Run

To run Section-LIO, execute the following commands:

```bash
# Source the environment
source ./catkin_ws/devel/setup.bash

# For m2dgr
roslaunch section-lio m2dgr.launch

# For avia
roslaunch section-lio avia.launch
```

## 4 Datasets

You can access our datasets at [Google Drive](https://drive.google.com/drive/folders/1izEMEl5SHUZ4EPXbKX7SNY4MzoQyJBSA?usp=sharing).