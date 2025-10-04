# Base image with ROS 2 Jazzy and desktop tools
FROM osrf/ros:jazzy-desktop
ENV DEBIAN_FRONTEND=noninteractive

# Install basic utilities
RUN apt-get update && apt-get install -y \
    git curl wget nano python3-pip \
    build-essential cmake python3-colcon-common-extensions \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic + ROS 2 Jazzy integration
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-hardware-interface \
    ros-jazzy-pluginlib \
    ros-jazzy-rclcpp \
    ros-jazzy-yaml-cpp-vendor \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-twist-mux \
    ros-jazzy-rviz-visual-tools \
    && rm -rf /var/lib/apt/lists/*

# Create workspace and clone repositories
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

RUN git clone -b jazzy https://github.com/gazebosim/ros_gz.git
RUN git clone -b jazzy https://github.com/ros-controls/gz_ros2_control.git
RUN git clone https://github.com/ros-visualization/rqt_robot_steering.git
RUN git clone https://github.com/ElephantRobotics/mycobot_ros2.git

# Install dependencies using rosdep
WORKDIR /root/ros2_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys="python-tk" && \
    rm -rf /var/lib/apt/lists/*

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Configure environment and aliases
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias build='cd ~/ros2_ws && colcon build && source install/setup.bash'" >> /root/.bashrc

CMD ["bash"]