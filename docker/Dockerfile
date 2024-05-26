# Use the the Ros2 Humble Desktop image as the base image
FROM osrf/ros:humble-desktop-full

# Set display environment variable
ENV DISPLAY=:0

# Set the working directory
WORKDIR /root

# Running commands
# installing new updates, gazebo, ros2 cartographer and navigation stack
# install the turtlebot packages from source like we did in ros1 AMR
RUN apt-get update \
    && apt-get install -y \
      ros-humble-gazebo-* \
      ros-humble-ros-gz \
      vim \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-cartographer \
      ros-humble-cartographer-ros \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-ament-cmake-clang-format \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p ~/ros2_ws/src/ \
    && cd ~/ros2_ws/src \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && cd ~/ros2_ws \
    && /bin/bash -c "source /opt/ros/humble/setup.sh && source /opt/ros/humble/setup.bash && colcon build --symlink-install" \
    && echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc \
    && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
    && echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c "source ~/.bashrc" \