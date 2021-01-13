#  astra_ros

<inline-page://homeastra_rosn-astra_ros>

astra_ros wraps the Orbbec Astra SDK for ROS. It can be used as a library (with or without ROS), a nodelet, or a node.

## 1. Installation

```.sh
cd /path/to/your_catkin_workspace/src
git clone https://github.com/semio-ai/astra_ros.git
```

## 2. Compilation

The Astra SDK must be installed on the system. Set the environment variable `ASTRA_ROOT` to the root of the Astra SDK if it cannot be found.

```.sh
cd /path/to/your_catkin_workspace
export ASTRA_ROOT=/path/to/astra_sdk
catkin_make
```

## 3. Documentation

### 3.1. Node Tutorial

View the node tutorial at <https://github.com/semio-ai/astra_ros/wiki/Tutorial>.

### 3.2. API Documentation

View the C++ API documentation at <https://semio-ai.github.io/astra_ros/annotated.html>.

## 4. ROS Node

### 4.1. Parameters

  - `~/devices` - A map of devices to open and publish
  - `~/devices/$DEVICE_NAME` - Parameters for the device with the name `$DEVICE_NAME`. `$DEVICE_NAME` will determine topic namespacing.

#### 4.1.1. Color Stream
  - `~/devices/$DEVICE_NAME/color` - Must be present for the color stream to be enabled. Can either contain sub-keys or be set to `true`. If the stream is not enabled, setting it to *running* later is not possible.
  - `~/devices/$DEVICE_NAME/color/running` - Set whether the color stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/color/mirrored` - Set whether the color frames are mirrored horizontally. Defaults to `false`.

#### 4.1.2. IR Stream
  - `~/devices/$DEVICE_NAME/ir` - Must be present for the IR stream to be enabled. Can either contain sub-keys or be set to `true`. If the stream is not enabled, setting it to *running* later is not possible.
  - `~/devices/$DEVICE_NAME/ir/running` - Set whether the IR stream is running. Defaults to `true`.

#### 4.1.3. Depth Stream
  - `~/devices/$DEVICE_NAME/depth` - Must be present for the depth stream to be enabled. Can either contain sub-keys or be set to `true`. If the stream is not enabled, setting it to *running* later is not possible.
  - `~/devices/$DEVICE_NAME/depth/running` - Set whether the depth stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/depth/mirrored` - Set whether the depth frames are mirrored horizontally. Defaults to `false`.

#### 4.1.4. Body Stream
  - `~/devices/$DEVICE_NAME/body` - Must be present for the body stream to be *enabled*. Can either contain sub-keys or be set to `true`. If the stream is not enabled, setting it to *running* later is not possible.
  - `~/devices/$DEVICE_NAME/body/running` - Set whether the body stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/body/publish_body_markers` - Set whether visualization markers are published for tracked bodies. Defaults to `false`.
  - `~/devices/$DEVICE_NAME/body/publish_body_mask` - Set whether the body mask image is published. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/body/publish_floor_mask` - Set whether the floor mask image is published. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/body/license` - The license string for the Orbbec Body Tracking SDK. Depending on the SDK version used, this value must be set.
  - `~/devices/$DEVICE_NAME/body/frame_id` - Must be set to the orbbec camera's optical frame.

#### 4.1.5. Hand Stream
  - `~/devices/$DEVICE_NAME/hand` - Must be present for the hand stream to be enabled. Can either contain sub-keys or be set to `true`. If the stream is not enabled, setting it to *running* later is not possible.
  - `~/devices/$DEVICE_NAME/hand/running` - Set whether the hand stream is running. Defaults to `true`.

#### 4.1.6. Masked Color Stream
  - `~/devices/$DEVICE_NAME/masked_color` - Must be present for the masked color stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/masked_color/running` - Set whether the masked color stream is running. Defaults to `true`.

#### 4.1.7. Colorized Body Stream
  - `~/devices/$DEVICE_NAME/colorized_body` - Must be present for the colorized body stream to be enabled. Can either contain sub-keys or be set to `true`. If the stream is not enabled, setting it to *running* later is not possible.
  - `~/devices/$DEVICE_NAME/colorized_body/running` - Set whether the colorized body stream is running. Defaults to `true`.

#### 4.1.8. Point Stream
  - `~/devices/$DEVICE_NAME/point` - Must be present for the point stream to be enabled. Can either contain sub-keys or be set to `true`. If the stream is not enabled, setting it to *running* later is not possible.
  - `~/devices/$DEVICE_NAME/point/running` - Set whether the point stream is running. Defaults to `true`.

#### 4.1.9. Example ROS Parameter file

```.yaml
devices:
  default:
    color: true
    depth: true
    ir: false # The Astra SDK only supports two "physical" streams from the sensor at once!
    body:
      publish_body_markers: true
      frame_id: "quori/head_camera_optical"
      license: "$YOUR_LICENSE"
```

### 4.2. Topics

#### 4.2.1. Color Stream
  - `color/image_color: sensor_msgs/Image` - The color image
  - `color/camera_info: sensors_msgs/CameraInfo` - The color camera's intrinsics

#### 4.2.2. IR Stream
  - `ir/image: sensor_msgs/Image` - The IR image

#### 4.2.3. Depth Stream
  - `depth/image: sensor_msgs/Image` - The depth image
  - `depth/camera_info: sensors_msgs/CameraInfo` - The depth camera's intrinsics

#### 4.2.4. Body Stream
  - `body/frame: astra_ros/BodyFrame` - The body frames
  - `body/markers: visualization_msgs/MarkerArray` - The body markers
  - `body/mask: sensor_msgs/Image` - The body mask
  - `body/floor_mask: sensor_msgs/Image` - The floor mask

#### 4.2.5. Colorized Body Stream
  - `colorized_body/image: sensor_msgs/Image` - The colorized body image

#### 4.2.6. Masked Color Stream
  - `masked_color/image: sensor_msgs/Image` - The masked color image

#### 4.2.7. Point Stream
  - `point_cloud: sensor_msgs/PointCloud` - The RGBD point cloud


### 4.3. Services

#### 4.3.1. Color Stream
  - `color/get_usb_info: astra_ros/UsbInfo` - Get the USB information for the color camera.
  - `color/get_mirrored: astra_ros/GetMirrored` - Get whether the color frame is mirrored horizontally.
  - `color/set_mirrored: astra_ros/SetMirrored` - Set whether the color frame is mirrored horizontally.
  - `color/get_image_stream_modes: astra_ros/GetImageStreamModes` - Get the image stream modes supported by the color camera.
  - `color/get_image_stream_mode: astra_ros/GetImageStreamMode` - Get the current image stream mode used by the color camera.
  - `color/set_image_stream_mode: astra_ros/SetImageStreamMode` - Configure the image stream mode for the color camera.
  - `color/get_running: astra_ros/GetRunning` - Get the running status of the color camera.
  - `color/set_running: astra_ros/SetRunning` - Set the running status of the color camera. False means that no color frames will be published.

#### 4.3.2. Depth Stream
  - `depth/get_chip_id: astra_ros/GetChipId` - Get the Orbbec chip ID.
  - `depth/get_mirrored: astra_ros/GetMirrored` - Get whether the depth frame is mirrored horizontally.
  - `depth/set_mirrored: astra_ros/SetMirrored` - Set whether the depth frame is mirrored horizontally.
  - `depth/get_registration: astra_ros/GetRegistration` - Get whether the depth frame is registered.
  - `depth/set_registration: astra_ros/SetRegistration` - Set whether the depth frame is registered.
  - `depth/get_usb_info: astra_ros/UsbInfo` - Get the USB information for the depth camera.
  - `depth/get_image_stream_modes: astra_ros/GetImageStreamModes` - Get the image stream modes supported by the depth camera.
  - `depth/get_image_stream_mode: astra_ros/GetImageStreamMode` - Get the current image stream mode used by the depth camera.
  - `depth/set_image_stream_mode: astra_ros/SetImageStreamMode` - Configure the image stream mode for the depth camera.
  - `depth/get_running: astra_ros/GetRunning` - Get the running status of the depth camera.
  - `depth/set_running: astra_ros/SetRunning` - Set the running status of the depth camera. False means that no depth frames will be published.

#### 4.3.3. IR Stream
  - `ir/get_usb_info: astra_ros/UsbInfo` - Get the USB information for the IR camera
  - `ir/get_image_stream_modes: astra_ros/GetImageStreamModes` - Get the image stream modes supported by the IR camera
  - `ir/get_image_stream_mode: astra_ros/GetImageStreamMode` - Get the current image stream mode used by the IR camera
  - `ir/set_image_stream_mode: astra_ros/SetImageStreamMode` - Configure the image stream mode for the IR camera
  - `ir/get_running: astra_ros/GetRunning` - Get the running status of the IR camera
  - `ir/set_running: astra_ros/SetRunning` - Set the running status of the IR camera. False means that no IR frames will be published.

#### 4.3.4. Colorized Body Stream
  - `colorized_body/get_running: astra_ros/GetRunning` - Get the running status of the colorized body stream
  - `colorized_body/set_running: astra_ros/SetRunning` - Set the running status of the colorized body stream. False means that no colorized body frames will be published.

#### 4.3.5. Hand Stream
  - `hand/get_running: astra_ros/GetRunning` - Get the running status of the hand stream
  - `hand/set_running: astra_ros/SetRunning` - Set the running status of the hand stream. False means that no hand frames will be published.

#### 4.3.6. Point Stream
  - `point/get_running: astra_ros/GetRunning` - Get the running status of the point stream
  - `point/set_running: astra_ros/SetRunning` - Set the running status of the point stream. False means that no point frames will be published.

## 5. License

`astra_ros` is released under the terms of the 3-Clause BSD License. See [`LICENSE`](https://github.com/semio-ai/astra_ros/blob/master/LICENSE) for details.
