# kinova-cinema-bot

A ros2 package that runs off the Kinova Kortex API to simulate a 7-DOF arm that
centers a camera on the end effector (EE). 

## Setup

### Cloning

This project contains submodules - notably the Kinova-kortex2 API. When
initially the repository use the --recurse-submodules command.

```
git clone --recurse-submodules git@github.com:julakshah/kinova-cinema-bot.git
```


Alternatively, if you've already cloned the repository using git clone, you can
run the following command to ensure you've got the submodules locally.

```
git submodule update --init
```

### Dependencies

## Running

```
ros2 launch
```
