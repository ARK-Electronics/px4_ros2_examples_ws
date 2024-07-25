# ROS2 & PX4 CustomMode example
This tutorial is a basic example on how to utilize custom modes in PX4 using ROS2 and QGC. We would like to provide you with a basic example code that you can customize and utilize for your application.

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
Navigate to the directory you would like to place the worskpace and then run the following
```
clone
```
Then navigate into the workspace:
```
cd
```
Install the submoduls
```
init
```
Build the workspace
```
build
```
Source the workspace
```
source
```

### Run the example

#### Run the simulation environment

```
cd PX4-Autopilot/
make px4_sitl
```

#### Run the Micro XRCE-DDS Agent for the communication stream
```
MicroXRCEAgent udp4 -p 8888
```

#### Run QGC Daily build
```
cd run
```
Take off with the drone using the GUI

#### Launch your custom mode

```
instructons here
```

#### Start it from QGC
You can just start the custom node from the GUI or you can also map it to you

#### Closing remarks
ONce you are done do not forget to close all your terminals





## Video

## ARK Promo
For more open-source drone-related material, follow us on LinkedIn and Twitter:

[LinkedIn](https://www.linkedin.com/company/ark-electronics-llc/)

[X](https://x.com/ark_electr0nics)

If you are interested in US manufactored drone hardware plase check out out webpage:

[ARK Electronics](https://arkelectron.com/)
