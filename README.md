# HUSKY_COAL_MINE_SIM
A ROS package that implements a customized Husky UGV in a Gazebo simulation of an underground coal mine environment.

Disclaimer: The package is a combination of existing open-source github packages and has a BSD-3-Clause license.

## The package contains

- [husky-coal-sim] Portable package for gazebo-rviz simulation of a Husky equipped with Ouster or VLP-16 LiDAR in a coal mine environment (this package can be installed and launched from catkin workspace).
- [ouster-description] cloned from [OUSTER-DESCRIPTION](https://github.com/clearpathrobotics/ouster_description.git) 
- [ouster-ros] cloned from [OUSTER-ROS](https://github.com/ouster-lidar/ouster-ros)

## This package is using concepts and files from

- [HUSKY](https://github.com/husky/husky.git) - License: BSD-3-Clause
- [HUSKY-LIO-SAM](https://github.com/FarzadAziziZade/Husky-LIO-SAM.git) - License: Not listed  
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git) - License: BSD-3-Clause 
- [OUSTER-ROS](https://github.com/ouster-lidar/ouster-ros.git) - License: BSD-3-Clause 
- [OUSTER-DESCRIPTION](https://github.com/clearpathrobotics/ouster_description.git) - License: BSD-3-Clause  

## Requirements

- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Gazebo](https://github.com/gazebosim/gz-sim)
- [Git-LFS](https://git-lfs.com/)


## Installation steps


### 1.Define the ROS distro you want to install & your work_space

Note: defining commonly used parameters as environment parameters allows for faster implementation as well as for portability of commands. Note that you can either set new environment parameters every time you open a commands windows (will not be available if you open another commands window) or set them as permanent enviroment parameters in the ~/.bashrc file (available to all commands windows)

```
export ROS_DISTRO=noetic #if ros already installed, this should be set --> check with 'echo $ROS_DISTRO' or 'printenv | grep ROS_*'
```

### 2.Install ROS dependencies & sensor descriptions/rospackages

A) ROS dependencies:
```
sudo apt-get install -y ros-$ROS_DISTRO-gazebo-*
sudo apt-get install -y ros-$ROS_DISTRO-navigation
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard
sudo apt-get install -y ros-$ROS_DISTRO-robot-localization
sudo apt-get install -y ros-$ROS_DISTRO-robot-state-publisher
```
B) LiDAR descriptions/rospackages & their additional dependencies:
```
sudo apt-get install -y ros-$ROS_DISTRO-velodyne-* # make sure ros-$ROS_DISTRO-velodyne-pcl is installed
sudo apt install -y ros-$ROS_DISTRO-pcl-ros
sudo apt install -y ros-$ROS_DISTRO-tf2-geometry-msgs
sudo apt install -y ros-$ROS_DISTRO-rviz
sudo apt-get install -y libpcap-dev
sudo apt-get install -y libspdlog-dev
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake
```

C) HUSKY packages - more information at [HUSKY](http://wiki.ros.org/Robots/Husky)

```
sudo apt-get install ros-$ROS_DISTRO-husky-* 

```

### 3. Customize your gazebo world and husky robot

A) Download the files of this github repo (here it is downloaded into a workspace named ~/catkin_ws_coal_sim:
```
ws=~/catkin_ws_coal_sim
echo $ws  #check that the parameter exist and is set correctly
mkdir -p "${ws}/src"
cd "${ws}/src"
git-lfs clone https://github.com/billisandr/husky_coal_mine_sim.git
```

B) Build and source your workspace:
```
cd ${ws}
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source ${ws}/devel/setup.bash
```

C) Set environment parameters for simulation environment and robot: 
Note 1: These parameters are set to the default values when catkin_make building through the 'env-hooks' files inside the husky-coal-description and husky-coal-gazebo directories. Use 'echo' to check or use the following to set to desired values:
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find husky_coal_gazebo)/worlds
export HUSKY_DESCRIPTION=$(rospack find husky_coal_description)/urdf/husky_OS1-64_arch.urdf.xacro
#export HUSKY_GAZEBO_DESCRIPTION=${ws}/src/husky_coal_sim/husky_coal_description/urdf/husky_OS1-64_arch.urdf.xacro

printenv | grep -e GAZEBO_* -e HUSKY_*  # check environment params
```

Note 2: Check coal mine world only in gazebo with:
```
roslaunch husky_coal_gazebo coal_mine_playpen.launch
```

Note 3: Check robot only in gazebo with:
```
roslaunch husky_coal_gazebo husky_empty_world.launch
```

Note 4: The available robots in this package are stored in the /husky_coal_description/urdf directory and include:
i)husky_OS1-64_arch.urdf.xacro -> Husky with box (drone house) and an Ouster OS1-64 LiDAR on a rail/arch - (mostly used and most updated version)
ii)husky_1vel_arch.urdf.xacro -> Husky with box (drone house) and a Velodyne VLP-16 LiDAR on a rail/arch
iii)husky_4vel.urdf.xacro -> Husky with box (drone house) and 4 Velodyne VLP-16 LiDAR
iv)husky_4vel.urdf.xacro -> Husky with a Velodyne VLP-16 LiDAR

## How to run the simulations

### 1. (Optional) Various configurations through environment params

A) You may set the proper environment parameters to turn on/off sensors, bumpers, etc. See instructions at (https://github.com/husky/husky/tree/noetic-devel/husky_description)
```
export HUSKY_FRONT_BUMPER=1 #bumpers
export HUSKY_REAR_BUMPER=1
export HUSKY_SENSOR_ARCH_HEIGHT 510
export HUSKY_SENSOR_ARCH_OFFSET 0 -20 0
export HUSKY_LASER_3D_ENABLED=1
#export HUSKY_LASER_3D_TOPIC='ouster_cloud'
#export HUSKY_LASER_3D_HOST='192.168.131.20'
export HUSKY_LASER_3D_TOWER=1
export HUSKY_LASER_3D_PREFIX=''
export HUSKY_LASER_3D_PARENT='top_plate_link'
export HUSKY_LASER_3D_XYZ='0 0 0'
export HUSKY_LASER_3D_RPY='0 0 0'
```

B) Configure params for rviz visualization:
Default behavior of this package is that if HUSKY_RVIZ is true, rviz visualization is opened up alongside the gazebo simulation with the configuration file specified in HUSKY_RVIZ_CONF environment params. The second parameter is the name of the rviz file which must be located inside the /husky_coal_viz/rviz directory.

Default values are specified in /husky_coal_viz/env-hooks directory as 
export HUSKY_RVIZ=1  &
export HUSKY_RVIZ_CONF=husky_coal_robot_nav2.rviz. 

If users wish to change these values, they can do so in terminal:
```
export HUSKY_RVIZ=0
export HUSKY_RVIZ_CONF=<rviz_config_filename.rviz>
```

C) Define mapping and navigation packages:
Default navigation of this package is to use the gmapping and move-base packages from ros-$ROS_DISTRO-navigation package (launch from the husky_coal_navigation directory). Bby default the environment parameter that trigger this is set as: HUSKY_GMAP=1:
```
export HUSKY_GMAP=1
```

### 2.Launch the gazebo simulation

1. Make sure to source the desired catkin_workspace: 
``` 
source devel/setup.bash 
```
2. Make sure the environment parameter for the world ($GAZEBO_MODEL_PATH) and robot ($HUSKY_DESCRIPTION) are correct:
```
printenv | grep -e GAZEBO_* -e HUSKY_*
```

3. Launch simulation:
```
roslaunch husky_coal_gazebo husky_coalpen.launch
```

