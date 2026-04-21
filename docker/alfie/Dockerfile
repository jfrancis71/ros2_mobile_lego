FROM ros2_brickpi3
RUN apt install -y ros-jazzy-bicycle-steering-controller
RUN git -C src clone https://github.com/jfrancis71/ros2_mobile_lego
RUN colcon build --symlink-install --packages-select=alfie
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch alfie motors_launch.py"
