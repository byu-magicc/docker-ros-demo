# Base docker image. ros:${ROS_DISTRO} is a headless ROS image, which is smaller
# in size but does not have any GUI tools. osrf/ros:${ROS_DISTRO}-desktop-full
# is a larger image with most of ROS's GUI tools. Use the larger image if you
# want GUI tools.
ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}
# FROM osrf/ros:${ROS_DISTRO}-desktop-full 

# Update system
RUN apt update
RUN apt upgrade -y
RUN rosdep update

# Clone ROSplane and ROSflight repos
WORKDIR /rosflight_ws/src
RUN git clone --recursive https://github.com/rosflight/rosflight_ros_pkgs.git
RUN git clone https://github.com/rosflight/rosplane.git

# Install ROS dependencies with rosdep dependencies and build packages
WORKDIR /rosflight_ws
RUN rosdep install --from-paths . --ignore-src -y
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Add source files to .bashrc for automatic sourcing
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /rosflight_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc
ENTRYPOINT bash

