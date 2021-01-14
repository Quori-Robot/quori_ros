# quori_holonomic_drive_controller

A `controller_manager` plugin for controlling the RAMSIS mobile base in a psuedo-holonomic way.

See `quori_controller/config/quori_control_holo.yaml` for an example of using this plugin with the ROS controller stack.

## Parameters

### `~publish_rate: double` 
The rate at which velocities will be generated (default: `50.0`).

### `~open_loop: bool`
Estimate odometry internally rather than with encoder feedback (default: `false`)

### `~odom_frame_id: string`
The tf frame for odometry (default: `odom`)

### `~base_frame_id: string`
The base link for the ramsis (default: `ramsis/wheel_axle`)

### `~cmd_vel_timeout: double`
If a `cmd_vel` message has not been sent in this many seconds, the ramsis base is stopped (default: `0.5`).

### `~max_motor_left_vel: double`
The maximum left motor velocity in meters per second (default: `0.6`).

### `~max_motor_right_vel: double`
The maximum right motor velocity in meters per second (default: `0.6`).

### `~max_motor_turret_vel: double`
The maximum turret motor velocity in radians per second (default: `3.49`).

### `~wheel_separation: double`
The distance between the wheels, in meters (default: `0.12284075`).

### `~wheel_radius: double`
The radius of the wheels, in meters (default: `0.0762`).

## Subscribed Topics

### `cmd_vel`

Holonomic Twist messages used as input.