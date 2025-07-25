# ROS2 Mobile Lego Robots

Demos of ROS2 enabled Lego EV3 mobile robots (on a Raspberry Pi with BrickPi3 interface)

This repository does not really contain any software as such (apart from configuration and launch files). It is primarily about demonstrating how to configure existing ROS2 control components to work together to create a working robot. It does contain examples of lego designs for corresponding robots.

## <B>Guest Starring:</B>

|Name|As seen on YouTube|That's Me|
|------------------|----|----|
[Thomas](./thomas/README.md)|<a href="https://youtu.be/mzJLYzwhiqo"><img src="https://img.youtube.com/vi/mzJLYzwhiqo/0.jpg" height=320></a>|<img src=./thomas/images/final_assembly/step_10.jpg height=320>|
[Kitt](./kitt/README.md)||<img src=./kitt/images/final_assembly/step_11.jpg height=200>|
[Alfie](./alfie/README.md)||<img src=./alfie/images/final_assembly/step_7.jpg height=320>|



### Tested Hardware

Raspberry Pi 3 Model B+, Dexter Industries BrickPi3

### Tested Software

Ubuntu 22.04, ROS2 Jazzy (RoboStack), BrickPi3


## Installation

All these robots are based on BrickPi3 hardware so: follow instructions to build ROS2 BrickPi3 at (https://github.com/jfrancis71/ros2_brickpi3)

If environment not activated (eg you have logged in again since performing above step), ensure environment is activated:

```mamba activate ros2```


```
cd ~/ros2_ws
git -C src clone https://github.com/jfrancis71/ros2_mobile_lego.git
```

For all robots:
```
colcon build --symlink-install
```

### Troubleshooting

I recommend having a seperate shell running htop so you can monitor progress. The Raspberry Pi 3B+ is quite memory limited which can cause problems installing some packages, particularly the omni_wheel_controller in the last step above. If you see process status 'D' in htop relating to the install processes that persists this can indicate difficulties due to low memory. In this case I suggest before running the colcon build step (the final step in the instructions), adding:

```
export MAKEFLAGS="-j 1" # recommended to reduce memory usage.
```

Also I suggest adding some temporary swap (I found 2GB perfectly sufficient). See discussion from Digital Ocean in the References section. Don't forget to remove the swap after a succesful installation. (A swap file on an SD card will reduce card life significantly)


## Verify Install

To reinitialize environment:
```
source ./install/setup.bash
```


Activate the motor controller (replacing PACKAGE_NAME with your robot, eg thomas):
```
ros2 launch PACKAGE_NAME brickpi3_motors_launch.py
```

Note this command blocks the terminal, so for subsequent commands you will need another terminal.

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
