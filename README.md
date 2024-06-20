# ROS2 workspace for PX4

## Pre-requisites
```
git submodule update --init --recursive
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build
```

```
sudo apt-get install -y \
	ros-humble-usb-cam \
	ros-humble-image-view \
	ros-humble-camera-calibration \
	ros-humble-camera-calibration-parsers

```

## Usage

USB camera node
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p camera_info_url:=/home/jake/Downloads/fisheye_calibration.ini
```
View the image
```
ros2 run image_view image_view --ros-args -r image:=/image_proc
```

Bridge camera from gz to ros2

Install the proper version for humble + garden
```
sudo apt install ros-humble-ros-gzgarden-bridge
```
Run

```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

## Camera calibration
- fisheye calibration
- https://docs.ros.org/en/rolling/p/camera_calibration/tutorial_mono.html
USB camera
https://www.amazon.com/gp/product/B0829HZ3Q7/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
```
```
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 --ros-args -r image:=/image_raw
```

bridge camera from gz to ros2
```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

Run this before build ros_gz from source
```
export GZ_VERSION=garden
```