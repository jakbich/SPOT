## Champ's Spot robot
This repository is based on [CHAMP](https://github.com/chvmp/champ) and [zoo](https://github.com/chvmp/robots) repositories. Tested on Ubuntu Focal 20.04 (ROS Noetic).

## Installation

### CHAMP installation

Clone and install all dependencies:
```
sudo apt install -y python3-rosdep
cd <your_ws>/src
git clone --recursive https://github.com/chvmp/champ
git clone https://github.com/chvmp/champ_teleop
cd ..
rosdep install --from-paths src --ignore-src -r -y
```
### Spot robot simulation

You need a git SSH key setup. Navigate to `src` directory in your workspace, then:
```
 git clone https://github.com/Thomahawkuru/champ_spot.git
 cd champ_spot
 git submodule init
 git submodule update
```

### Build your workspace

```
 cd <your_ws>
 catkin build
 source <your_ws>/devel/setup.bash
```

### Gazebo Worlds
Add `models` directory to the GAZEBO_MODEL_PATH environment variable. You can add the following line to the end of your ~/.bashrc:
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to>/models
```
Examples: https://github.com/leonhartyao/gazebo_models_worlds_collection

### Quick Start Guide

Run the Gazebo environment:
```
roslaunch spot_config spawn_world.launch
```

Spawn the robot:
```
roslaunch spot_config spawn_robot.launch rviz:=true
```

In the simulation, only robot's front cameras are available. Note that the cameras are mounted sideways, so they have a narrower horizontal FoV, but a larger vertical one. The camera data is rotated anticlockwise by 90 degrees.

## Installing Spot ROS driver
In order to install the driver, follow the README instructions from [Boston Dynamic's Spot](https://github.com/clearpathrobotics/spot_ros) repository.
