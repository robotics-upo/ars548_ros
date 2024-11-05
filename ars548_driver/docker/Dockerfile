FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y git python3-colcon-common-extensions ros-humble-rviz2 ros-humble-tf2-ros libtclap-dev iproute2 net-tools
ARG DOCKER_USER=default_user
RUN adduser $DOCKER_USER
USER $DOCKER_USER 
WORKDIR /home/$DOCKER_USER/ros2_ws
RUN mkdir -p /home/$DOCKER_USER/ros2_ws/src && cd /home/$DOCKER_USER/ros2_ws/src && git clone https://github.com/robotics-upo/ars548_ros.git
RUN cd /home/$DOCKER_USER/ros2_ws && source /opt/ros/humble/setup.bash && colcon build && source /home/$DOCKER_USER/ros2_ws/install/setup.bash
RUN echo "source /home/$DOCKER_USER/ros2_ws/install/setup.bash">> ~/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source install/setup.bash && ros2 launch ars548_driver ars548_launch.xml viz:=false"]
