# quori_teleop

`quori_teleop` allows teleoperation of Quori using an Xbox or Playstation style controller.

## Controller Mappings

### Left Arm Control (Hold Left Trigger)

- `l_shoulder_pitch` - Left Thumb Stick (Y axis)
- `l_shoulder_roll` - Left Thumb Stick (X axis)

Hold `A` for "fast" mode, which doubles the movement speed.

### Right Arm Control (Hold Right Trigger)

- `r_shoulder_pitch` - Right Thumb Stick (Y axis)
- `r_shoulder_roll` - Right Thumb Stick (X axis)

Hold `A` for "fast" mode, which doubles the movement speed.

### Base Control (Hold Left Bumper)

Move the holonomic ramsis base.

- Linear X - Left Thumb Stick (Y axis)
- Linear Y - Left Thumb Stick (X axis)
- Angular Z - Right Thumb Stick (X axis)

### Waise Hinge Control (Hold Right Bumper)

- `waste_pitch` - Left Thumb Stick (Y axis)

## Subscribed Topics

- `/joint_states` - Used for joint position feedback
- `/joy` - Joystick input from the ROS `joy` node.

## Published Topics

- `/quori/base_controller/cmd_vel` - The commanded velocity for the ramsis base
- `/quori/joint_trajectory_controller/command` - Joint trajectory commands for Quori's joints.
