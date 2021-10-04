# quori_controller

quori_controller is an implementation of the ROS control stack for Quori and the RAMSIS base.

## Usage

### `roslaunch quori_controller quori_control_holo.yaml`

Launches the ROS control stack for Quori and the RAMSIS base in holonomic mode. The turret joint will not be accessible to ROS, as it is used in generating the psuedoholonomic behavior.

### `roslaunch quori_controller quori_control_diff.yaml`

Launches the ROS control stack for Quori and the RAMSIS base in differential drive mode. The turret joint will be accessible via ROS.

### Calibration and `calibration.yaml`
The base turret orientation is calibrated with the `calibration.yaml` file. Open the file and set `base_offset:` to your robot's correct value. This value can be updated later if you update your hardware. To learn about calibrating the arms and spine see [quori_embedded](https://github.com/semio-ai/quori_embedded/blob/master/README.md)
