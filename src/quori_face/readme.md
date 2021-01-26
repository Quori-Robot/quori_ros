# quori_face

`quori_face` accepts an image and displays it on the Quori face. The image is treated as containing radial pixels along the polar and azimuthal axes of the spherical face.

## Calibration

To calibrate, run `roslaunch quori_face calibrate.launch`. This will spawn `quori_face_node` with a grid image and a dynamic_reconfigure user interface for adjusting the calibration. At exit, the new calibration will be written to `quori_face/config/params.yaml`.

## Usage

To use, run `roslaunch quori_face quori_face.launch`.

## Parameters

### Transform Parameters

These parameter names directly correspond to the paper "LOW COST OPTICAL MECHANICAL SYSTEM FOR HUMAN ROBOT INTERACTION" (IMECE2018-87885) on the face projection transform algorithm.

- `~transform/R: double` (default: `4.0`) - Radius of the spherical head.
- `~transform/r_m: double` (default: `1.5`) - Radius of the mirror.
- `~transform/r_o: double` (default: `2.0`) - Radius of the base circle at the bottom of the head.
- `~transform/h: double` (default: `1.0`) - Distance from the center of projection to the bottom of the head.
- `~transform/L: double` (default: `8.4476252817457738`) - The distance between the projection center and the intersection of the projection axis and the sphere.
- `~transform/epsilon: double` (default: `0.059068559067049511`) - The angle between the axis of roation of the circle and the intersection of the projection axis and the sphere.
- `~transform/delta/x: double` (default: `-2.9`) - X-axis projector installation error.
- `~transform/delta/y: double` (default: `-1.6`) - Y-axis projector installation error.
- `~transform/screen_size/x: uint32` (default: `1920`) - Width of the projector screen in pixels.
- `~transform/screen_size/y: uint32` (default: `1080`) - Height of the projector screen in pixels.

### Lookup Table Parameters

`quori_face` aggressively caches the expensive transform algorithm into a lookup table.

- `~lookup_resolution/x: uint32` (default: `2048`) - Width of the transform lookup table in pixels.
- `~lookup_resolution/y: uint32` (default: `2048`) - Height of the transform lookup table in pixels.

### Image Parameters

The input image must be of fixed width and height during the execution of `quori_face_node`. If the received image is of a different size, it is resized before being transform. For optimal performance, this value should match the expected input image resolution.

- `~image_resolution/x: uint32` (default: `1280`) - Expected width of received images for display.
- `~image_resolution/y: uint32` (default: `720`) - Expected height of received images for display.

### Spherical Mapping Parameters

Quori may optionally be configured with a helmet. We can adjust the spherical region mapped to the input image by adjusting the `min` and `max` coordinates.

- `~center/theta: double` (default: `1280`) - Central polar value (that corresponds to the center of the image), in radians, to map.
- `~center/psi: double` (default: `720`) - Central azimuthal value (that corresponds to the center of the image), in radians, to map.
- `~min/theta: double` (default: `-PI * 0.45`) - Minimum polar value, in radians, to map.
- `~min/psi: double` (default: `-PI * 0.5`) - Minimum azimuthal value, in radians, to map.
- `~max/theta: double` (default: `PI * 0.45`) - Maximum polar value, in radians, to map.
- `~max/psi: double` (default: `PI * 0.5`) - Maximum azimuthal value, in radians, to map.

## Dynamic Reconfigure Options

`quori_face` allows certain ROS parameters to be adjusted on-the-fly for easy calibration.

- `dx: double` - The `~transform/delta/x` value (see above). 
- `dy: double` - The `~transform/delta/y` value (see above).

## Subscribed Topics

- `image: sensor_msgs/Image` - The image to display on the face. The image is treated as being composed of radial pixels. For optimal performance, the image should match the configured `~image_resolution` and be BGR8 encoded.



