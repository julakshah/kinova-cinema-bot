# kinova-cinema-bot

A ros2 package that runs off the Kinova Kortex API to simulate a 7-DOF arm that
centers a camera on the end effector (EE). 

## Challenges Faced

* **Mediapipe**

## Setup

The setup for this project assumes the system it is being run on is Ubuntu 22.04
- Jammy Jellyfish - and has ros2 Humble installed and sourced. 

For more information see the [ROS2 Humble installation page](https://docs.ros.org/en/humble/Installation.html)

```bash
source /opt/ros/humble/setup.bash
```

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

### Dependencies

* Ubuntu 22.04
* ROS2 Humble
* Google Mediapipe
* CMake & Colcon (Mediapipe aaaaaaah)


## Running

```
ros2 launch
```

## Resources & Sources
* [ros2_kortex api](https://github.com/Kinovarobotics/ros2_kortex/tree/humble?tab=readme-ov-file)
* [Gaurav Gupta Medium post on ROS Moveit Servo](https://medium.com/black-coffee-robotics/ros-moveit-servo-with-kinova-arm-9b0f4baa80c6)

### Mediapipe
* [Google Mediapipe C++ Reference](https://ai.google.dev/edge/mediapipe/framework/getting_started/cpp)
* [Google Mediapipe GPU Support](https://ai.google.dev/edge/mediapipe/framework/getting_started/gpu_support.md#opengl-es-setup-on-linux-desktop)
* [Mitya Shabat Medium Post for Mediapipe Through CMake](https://medium.com/@mitya.shabat/connecting-mediapipe-with-cmake-for-my-own-hand-tracking-app-f3e57dc14b8d)
* [Mitya Shabat Personal Post for Mediapipe Through Cmake](https://xallt.github.io/posts/connecting-mediapipe-cmake/)
* [Mitya Shabat Archived Github Project with Mediapipe](https://github.com/Xallt/HandTrackingProject)
