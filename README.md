# AKAZE Detector ROS2 Package

## Overview

This ROS2 package implemented an AKAZE feature detector node. The node subscribed to an image topic, processed the images using the AKAZE algorithm, and published the processed images with detected keypoints. The images were captured using an iPhone as a webcam with the help of Iriun for Linux and the `ros2_usb_camera` package.

![AKAZE Image 1](/docs/AKAZE_image_1.png)

![AKAZE Image 2](/docs/AKAZE_image_2.png)

## Process Description


### Using an iPhone as a Webcam

The process of using an iPhone as a webcam involved the following steps:

1. **Installed the Iriun Webcam App**:
   - The Iriun Webcam app was downloaded and installed from the App Store on the iPhone.

2. **Installed Iriun Webcam Software on Linux**:
   - The Iriun Webcam software for Linux was downloaded from Iriun's official website and installed following the provided instructions.

3. **Connected the iPhone**:
   - Both the iPhone and the Linux machine were connected via usb.
   - The Iriun Webcam app was opened on the iPhone, and the Iriun Webcam software was launched on the Linux machine, allowing the iPhone to be recognised as a webcam.

### Setting Up `ros2_usb_camera`

To capture the webcam footage, the following steps were taken:

1. **Installed `ros2_usb_camera` Package**:
   - The `ros2_usb_camera` repository was cloned into the ROS2 workspace:
     ```
     cd ~/ros2_ws/src
     git clone https://github.com/ros-drivers/ros2_usb_camera.git
     ```
   - The package was built:
     ```
     cd ~/ros2_ws
     colcon build --packages-select ros2_usb_camera
     ```

2. **Ran `ros2_usb_camera` Node**:
   - The workspace was sourced:
     ```
     source ~/ros2_ws/install/setup.bash
     ```
   - The `ros2_usb_camera` node was launched:
     ```
     ros2 run ros2_usb_camera ros2_usb_camera_node
     ```
   - This published the webcam footage to the `/image` topic.

### Implementing the AKAZE Feature Detector

The implementation of the AKAZE feature detector involved the following steps:

1. **Created the `akaze_detector` Package**:
   - A new ROS2 package named `akaze_detector` was created:
     ```
     ros2 pkg create --build-type ament_cmake akaze_detector --dependencies rclcpp sensor_msgs cv_bridge
     ```

2. **Wrote the AKAZE Implementation**:
   - A file named `akaze_detector.cpp` was created in the `src` directory of the `akaze_detector` package. This file contained the implementation of the node that subscribed to the `/image` topic, processed the images using AKAZE, and published the results to a new topic.

3. **Built the Package**:
   - The `akaze_detector` package was built:
     ```
     cd ~/ros2_ws
     colcon build --packages-select akaze_detector
     ```

4. **Ran the AKAZE Detector Node**:
   - The workspace was sourced:
     ```
     source ~/ros2_ws/install/setup.bash
     ```
   - The `akaze_detector_node` was run:
     ```
     ros2 run akaze_detector akaze_detector_node
     ```

## Visualising the Processed Images

The processed images with keypoints were visualised using `rqt_image_view`. `rqt` was opened, and the `Image View` plugin was added. The `akaze_image` topic was then selected to see the output
.`


[AKAZE Video](docs/AKAZE_Video.webm)