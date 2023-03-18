# lsn10_ros

## version track
Author: leo

### ver1.00_210826 leo
1. Add angle compensate
2. Add truncate angle area data

## Description
The `lsn10_ros` package is a linux ROS driver for lsn10.
The package is tested on Ubuntu 16.04 with ROS kinetic.

## revolution
open new terminal
rostopic pub -1 /rev_order std_msgs/Int8 "data: 6" 		(6hz)
.
.
.
rostopic pub -1 /rev_order std_msgs/Int8 "data: 12" 		(12hz)
It can only be adjusted to 6 to 12 Hz

rostopic pub -1 /rev_order std_msgs/Int8 "data: 1" 		( Add filter )

rostopic pub -1 /rev_order std_msgs/Int8 "data: 0" 		( End filter )



## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
sudo chmod 777 /dev/ttyUSB0
source devel/setup.bash
roslaunch lsn10 lsn10.launch 

```

**Published Topics**

``/scan`` (`sensor_msgs/scan`)

**Node**

```
roslaunch lsn10 lsn10.launch
```

## FAQ

## Bug Report







