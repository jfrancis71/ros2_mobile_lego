FROM ros2_brickpi3
RUN apt update
RUN apt install -y ros-jazzy-bicycle-steering-controller
RUN git -C src clone https://github.com/jfrancis71/ros2_mobile_lego
RUN colcon build --symlink-install --packages-select=alfie
ENV MAKEFLAGS="-j 1"
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select alfie_hardware"
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch alfie motors_launch.py"
