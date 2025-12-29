# kinova-cinema-bot

This ros2 package is an attempt to fuse Kinova's ros2_kortex api with Google's
mediapipe face landmark detection algorithm to autonomously control a robot arm
that is holding a camera.

This is the second phase of this project, after [kinova-kinematics](https://github.com/julakshah/kinova-kinematics),
where I learned to control a physical Gen 3 Lite robot arm through the python
API. However, the python api had several issues, foremost of which was the
control loop update speed was only 60Hz compared to the 1kHz for the Cpp api.
This project seeks help me familiarize myself with ROS2 C++, Kinova's
ros2_kortex package in C++, and google mediapipe in ROS2.

## Reflection & Challenges

Primary Challenges:

- Google Mediapipe C++ is poorly documented
- Google Mediapipe C++ requires bazel to launch
- Creating a ros2 package containing both python and C++ nodes
- Luanching MoveIt2 with the ros2_kortex API

## Setup

The setup for this project assumes the system it is being run on is Ubuntu 22.04

- Source ros2 "**source /opt/ros/humble/setup.bash**"

For more information see the [ROS2 Humble installation page](https://docs.ros.org/en/humble/Installation.html)

```bash
source /opt/ros/humble/setup.bash
```

### Dependencies

- Ubuntu 22.04
- ROS2 Humble
- Google Mediapipe
- CMake & Colcon

### Cloning

This project contains submodules. When initially the repository use the
--recurse-submodules command.

```
git clone --recurse-submodules git@github.com:julakshah/kinova-cinema-bot.git
```

Alternatively, if you've already cloned the repository using git clone, you can
run the following command to ensure you've got the submodules locally.

```
git submodule update --init
```

## Running

Please make sure you have two TTY's open.

```
ros2 launch
```

## Resources & Sources

- [ros2_kortex api](https://github.com/Kinovarobotics/ros2_kortex/tree/humble?tab=readme-ov-file)
- [Gaurav Gupta Medium post on ROS Moveit Servo](https://medium.com/black-coffee-robotics/ros-moveit-servo-with-kinova-arm-9b0f4baa80c6)

### Mediapipe

- [Google Mediapipe C++ Reference](https://ai.google.dev/edge/mediapipe/framework/getting_started/cpp)
- [Google Mediapipe GPU Support](https://ai.google.dev/edge/mediapipe/framework/getting_started/gpu_support.md#opengl-es-setup-on-linux-desktop)
- [Mitya Shabat Medium Post for Mediapipe Through CMake](https://medium.com/@mitya.shabat/connecting-mediapipe-with-cmake-for-my-own-hand-tracking-app-f3e57dc14b8d)
- [Mitya Shabat Personal Post for Mediapipe Through Cmake](https://xallt.github.io/posts/connecting-mediapipe-cmake/)
- [Mitya Shabat Archived Github Project with Mediapipe](https://github.com/Xallt/HandTrackingProject)
