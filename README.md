# ROS2 & PX4 CustomMode example
This tutorial is a example on how to utilize custom modes in PX4 using ROS2 and QGC. We would like to provide you with a basic example code that you can customize and utilize for your application.

This project is a customized example of the usage of the Auterions PX4-ROS2 Interface Library

https://github.com/Auterion/px4-ros2-interface-lib

### Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* PX4 Autopilot
* Micro XRCE-DDS Agent
* QGroundControl Daily Build 

You can find the required instructions collected below

https://docs.px4.io/main/en/ros2/user_guide.html


https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html


## Usage

### Setup the Workspace
Make sure you source ROS2 Humble in the terminal you are using.
```
source /opt/ros/humble/setup.bash
```
OR
Just add the line above to your bashrc, in that case it is going to be sourced every time you open a terminal.
```
nano ~/.bashrc
```


Navigate to the directory you would like to place the worskpace and then run the following
```
git clone https://github.com/ARK-Electronics/px4_ros2_examples_ws
```
Then navigate into the workspace:
```
cd px4_ros2_examples_ws
```
Install the submoduls
```
git submodule update --init --recursive
```
Build the workspace
```
colcon build
```
Source the workspace
```
source install/setup.bash 
```

### Run the example

#### Run the simulation environment

```
cd PX4-Autopilot/
make px4_sitl_default gz_x500
```

#### Run the Micro XRCE-DDS Agent for the communication stream
```
MicroXRCEAgent udp4 -p 8888
```

#### Run QGC Daily build
Navigate to the directory
```
./QGroundControl.AppImage
```
Take off with the drone using the GUI

#### Launch your custom mode
I created a launch file that you can use. It currently contains only one node, so it might seem limited, but you can expand on it. The file includes three basic patterns: circle, spiral, and figure-8. These are ROS2 parameters that you can set either directly in the launch file or via command line arguments. If no pattern is specified, the default is circle.

```
cd px4_ros2_examples_ws/
source install/setup.bash 
```
AND
```
ros2 run custom_mode custom_mode
```
OR
```
ros2 launch custom_mode custom_mode.launch.py
```
OR
```
ros2 launch custom_mode custom_mode.launch.py trajectory_type:=spiral
```
OR
```
ros2 run custom_mode custom_mode --ros-args -p trajectory_type:=figure_8

```

#### Start it from QGC
You can just start the custom node from the GUI or you can also map it to your remote control

#### Closing remarks
ONce you are done do not forget to close all your terminals

## Video

## ARK Electronics
For more open-source drone-related material, follow us on LinkedIn and Twitter:

[LinkedIn](https://www.linkedin.com/company/ark-electronics-llc/)

[X](https://x.com/ark_electr0nics)

If you're interested in US-manufactured drone hardware, please visit our webpage:

[ARK Electronics](https://arkelectron.com/)

## Questions
Message Patrik Dominik Pordi on the Dronecode Foundation Discord for questions or email me at patrik@arkelectron.com
