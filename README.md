# ROS2 Mobile Lego Robots

Demos of ROS2 enabled Lego EV3 mobile robots (on a Raspberry Pi with BrickPi3 interface)

This repository does not really contain any software as such (apart from configuration and launch files). It is primarily about demonstrating how to configure existing ROS2 control components to work together to create a working robot. It does contain examples of lego designs for corresponding robots.

## <B>Guest Starring:</B>

|Name|As seen on YouTube|That's Me|
|------------------|----|----|
[Thomas](./thomas/README.md)|<a href="https://youtu.be/mzJLYzwhiqo"><img src="https://img.youtube.com/vi/mzJLYzwhiqo/0.jpg" height=320></a>|<img src=./thomas/images/final_assembly/step_10.jpg height=320>|
[Kitt](./kitt/README.md)|<a href="https://youtu.be/JI3d6wAOGeA"><img src="https://img.youtube.com/vi/JI3d6wAOGeA/0.jpg" height=320></a>|<img src=./kitt/images/final_assembly/step_11.jpg height=200>|
[Alfie](./alfie/README.md)|<a href="https://youtu.be/7NEJ1teqLHA"><img src="https://img.youtube.com/vi/7NEJ1teqLHA/0.jpg" height=320></a>|<img src=./alfie/images/final_assembly/step_7.jpg height=320>|



### Tested Hardware

Raspberry Pi 3 Model B+, Dexter Industries BrickPi3

### Tested Software

Ubuntu 22.04, ROS2 Jazzy (RoboStack), BrickPi3


## Installation

All these robots are based on BrickPi3 hardware so: follow instructions to build ROS2 BrickPi3 at (https://github.com/jfrancis71/ros2_brickpi3)

```
git clone -b microservices https://github.com/jfrancis71/ros2_mobile_lego.git
```

e.g. to build Thomas: (otherwise replace with kitt/alfie)
```
docker build -t thomas ./ros2_mobile_lego/docker/thomas
```


### Troubleshooting

I suggest adding some temporary swap (I found 2GB perfectly sufficient). See discussion from Digital Ocean in the References section. Don't forget to remove the swap after a succesful installation. (A swap file on an SD card will reduce card life significantly)


## Verify Install

```
docker run -it --rm --network=host --ipc=host --device=/dev/spidev0.1 thomas
```

This should cause the motors to rotate (briefly):
```
ros2 topic pub -t 5 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

## Remote Control

To control by keyboard:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

To control by joystick:

You can use the ros2 teleop-twist-joy package to control by joystick. The repo https://github.com/jfrancis71/ros2_joystick_config/ contains examples for a XEOX Gamepad joystick.


## References:

Useful discussion on swap file on Ubuntu:
https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04


Dexter Industries BrickPi3:
https://www.dexterindustries.com/brickpi-core/


Book:
The Unofficial Lego Technic Builder's Guide, Pawet Sariel Kmiec


ROS2 Book: Robot Programming with ROS2, Francisco Martin Rico, 2023.


Robot Cheat sheet:
https://www.theroboticsspace.com/assets/article3/ros2_humble_cheat_sheet2.pdf
