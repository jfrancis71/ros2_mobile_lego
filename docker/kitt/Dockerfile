FROM ros2_brickpi3
RUN apt install -y ros-jazzy-tricycle-steering-controller
RUN git -C src clone https://github.com/jfrancis71/ros2_mobile_lego
RUN colcon build --symlink-install --packages-select=kitt
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch kitt motors_launch.py"
