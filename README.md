# quori_ros

This catkin workspace contains all of the ROS packages necessary for full operation of the Quori robot platform from UPenn.

## External Dependencies

Some ROS packages are required for use of this workspace:
```
sudo apt-get install ros-${ROS_DISTRO}-sound-play
```

## Usage

```
git clone https://github.com/Quori-ROS/quori_ros.git
cd quori_ros
git submodule init
catkin_make
```
