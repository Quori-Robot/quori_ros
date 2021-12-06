# quori_description

Contains the kinematic description in URDF format for both Quori and the RAMSIS base. Quori can be used with or without the RAMSIS base, so standalone and combined URDFs are provided. 

## Quori Standalone URDF (`urdf/quori_standalone.xacro`)

This URDF file allows one to use the Quori robot without the RAMSIS base attached.

### Joints

  - `waist_pitch` (revolute) - The waist joint.
  - `r_shoulder_pitch` (revolute) - The first joint of the right arm.
  - `r_shoulder_roll` (revolute) - The second joint of the right arm.
  - `l_shoulder_pitch` (revolute) - The first joint of the left arm.
  - `l_shoulder_roll` (revolute) - The second joint of the left arm.

### Notable Links

  - `quori/base_link` - The root of the Quori kinematic tree (**Note: This is not the root when using the RAMSIS base with Quori**)
  - `quori/head_camera` - The frame for the Orbbec Astra Mini.
  - `quori/head_mic` - The frame for the head-mounted ReSpeaker microphone array.

## RAMSIS Standalone URDF (`urdf/ramsis_standalone.xacro`)

This URDF file allows one to use the RAMSIS holonomic base without Quori attached.

### Joints

  - `l_wheel` (continuous) - The left wheel joint.
  - `r_wheel` (continuous) - The right wheel joint.
  - `turret` (continuous) - The rotational joint for the mounting plate on top of the RAMSIS base (typically occupied by Quori).

### Notable Links

  - `ramsis/base_link` - The root of the RAMSIS kinematic tree (**Note: This is the root when using both Quori and the RAMSIS base**).
  - `ramsis/base_laser_scanner` - The frame for the RAMSIS-mounted 2D LIDAR.
