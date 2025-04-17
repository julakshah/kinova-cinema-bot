# kinova-cinema-bot (simulator)
Simulator for kinova-cinema-bot project.

## Table of Contents

# Setup
This setup guide is for MacOS/Linux machines.

## Required python version and modules
* python: 3.5 <= 3.xx < 3.10
    * requires python lower than 3.10 because module 'collections' was altered. Proceed to [I have newer python](###I-have-wrong-python) for help before returning to continue if you have >3.10.

### I have newer python
If you have an older or newer version of python you can setup a virtual environment with specific a specific version of python using conda. If you don't already have conda installed [***follow these instructions***](https://www.anaconda.com/docs/getting-started/miniconda/install) to install.

Then create a virtual environment with conda in current directory with
```
conda create --name <my_env_name> python=3.9
```
before activating it with 
```
conda activate <my_env_name>
```
## ROS Sim Setup
The sim environment uses ros2 and gazebo. It has been tested with ros2 Humble and Gazebo Fortress however should (in theory) work with Jazzy and newer but it is not recommended. Please follow [these instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) for installing ros2 humble.

Once installed, source your ros2 installation before running the sim. It should look something like:
```
source /opt/ros/humble/setup.bash
```
but it depends on how ros2 was installed.

# Running the Simulation
** Make sure you have sourced your ros2 installation **

In the project directory, run the following command to begin the simulation.

```
ros2 launch kortex_description view_robot.launch.py robot_type:=gen3_lite dof:=6
```
# Kinematics
The first part of this project consists of creating custom forward/inverse kinematics for the robot.


