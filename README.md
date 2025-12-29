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

Please see reflection.md for the full reflection.

## Setup

This setup assumes the following system and software requirements. If you are
missing mediapipe look [HERE](https://pypi.org/project/mediapipe/). If you are
missing the Kortex API look [HERE](https://github.com/Kinovarobotics/ros2_kortex/tree/humble).

- Ubuntu 22.04
- ROS2 Humble
- Google Mediapipe Python
- Kinova Kortex API
- CMake & Colcon

Make sure to source your ros environment.

- Source ros2 "**source /opt/ros/humble/setup.bash**"

For more information see the [ROS2 Humble installation page](https://docs.ros.org/en/humble/Installation.html)

```bash
source /opt/ros/humble/setup.bash
```

### Cloning

Please clone as normal or using the following command in the desired directory.

```bash
git clone git@github.com:julakshah/kinova-cinema-bot.git
```

## Running

Please make sure you have three TTY's open.

in the ros2_ws bin, build and source the package 'cinema-bot' as you would any
other package.

In the first TTY run:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config sim.launch.py \
  use_sim_time:=true
```

In the second TTY run:

```bash
ros2 run cinema-bot vision.py
```

In the third TTY run:

```bash
ros2 run cinema-bot control
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
