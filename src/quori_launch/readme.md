# quori_launch

This ROS package contains launch files for easily starting the Quori robot in a variety of modes.

## Launch Files

### `roslaunch quori_launch demo.launch`

- Starts the ROS navigation stack
- Starts mapping and laser publishers
- Starts the ROS controller stack for Quori in holonomic mode

### `roslaunch quori_launch mapping.launch`

Starts the laser stack (including filtration) and SLAM system.

### `roslaunch quori_launch tf_base_laser.launch`

Publishes the laser's static transform.

### `roslaunch quori_launch filtered_laser.launch`

Publishes the laser filtration stack (used by `mapping.launch`).