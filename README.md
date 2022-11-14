# quori_ros

This catkin workspace contains all of the ROS packages necessary for full operation of the Quori robot platform from UPenn.

## Prerequisites

The package is compatible with **Ubuntu 16.04 / ROS Kinetic** and **Ubuntu 20.04 / ROS Noetic**, built with the **Catkin** system. Make sure you have the right environment configured. This documentation assumes you are using noetic, so replace it with kinetic if needed.

## Initial Setup

### Turn on the Robot

1. Flip the breaker
2. Make sure the inverter has the switch turned to |
3. Press the power button to turn PC power on

### Real Robot Software Setup (first time setting up the robot)

```
mkdir quori_files
cd quori_files
git clone https://github.com/CMU-RASL/quori_ros.git
cd quori_ros
git submodule init
sudo apt-get install ros-noetic-sound-play ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
sudo apt install python3-rosdep
sudo apt-get install python3-pip
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install python3-catkin-tools
catkin config --init
catkin_build
source devel/setup.sh
```

In every new terminal
```
cd quori_files/quori_ros
source devel/setup.sh
```

### Simulation Software Setup (first time setting up simulation environment)

```
mkdir quori_files
cd quori_files
git clone https://github.com/CMU-RASL/quori_ros.git
cd quori_ros
git submodule init
sudo apt-get install ros-noetic-sound-play ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
sudo apt install python3-rosdep
sudo apt-get install python3-pip
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install python3-catkin-tools

sudo apt-get install libglfw3 libglfw3-dev


catkin config --init
catkin config --skiplist ros_astra_camera astra_ros quori_face
catkin_build
source devel/setup.bash
```

In every new terminal
```
cd quori_files/quori_ros
source devel/setup.bash
```

## Move Joints

### For Robot

Terminal 1: `roslaunch quori_controller quori_control_diff.launch`

Terminal 2: `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`

Alternate Terminal 2 (to be used with controller, keybindings at https://github.com/Quori-ROS/quori_ros/tree/master/src/quori_teleop): `roslaunch quori_teleop quori_teleop.launch`

### For Simulation

Terminal 1: `roslaunch quori_gazebo quori_world.launch`

Terminal 2: `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`

## Run the Face

This will launch the simulation with the face image embedded in the Rviz interface

Terminal 1: `roslaunch quori_gazebo quori_world.launch`

Terminal 2:
```
cd src/quori_face_generator/gui
python3 -m http.server 8000
```

Open a browser at `http://localhost:8000/FaceMotion mand3l.html`




