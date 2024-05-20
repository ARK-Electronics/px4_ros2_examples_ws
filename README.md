## ROS2 workspace for PX4

Getting started
```
git submodule update --init --recursive
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build
```