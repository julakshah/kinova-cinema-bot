# kinova-cinema-bot
Using a Kinova Gen3 Lite robot arm with an insta360 camera attached, implementation of custom control scheme and CV head tracking to smoothly track heads.

## Table of Contents
* kinova-cinema-bot
    * Project Structure and History
* Setup
    * Required python version and modules
        * I have newer python
    * Installing Kortex Python API (and dependencies)
    * Reference
* Kinematics

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

## Installing Kortex Python API (and dependencies)
Install what is needed to run the examples via a downloaded whl file (Python wheel package).

The whl file can be downloaded via the Kinova Artifactory: [kortex_api](https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl)  

On Linux:

```sh
python3 -m pip install <whl relative fullpath name>.whl
```
**Note:** root privilege is usually required to install a new module under Linux.

## Reference
* [Kinova API Github repo/documentation](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L)
* [Kinova Gen3 Lite robot arm manual](https://artifactory.kinovaapps.com/artifactory/generic-documentation-public/Documentation/Gen3%20lite/Technical%20documentation/User%20Guide/Gen3_lite_USER_GUIDE_R03.pdf)

# Kinematics
The first part of this project consists of creating custom forward/inverse kinematics for the robot.


