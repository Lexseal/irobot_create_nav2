# docker run -it --network host --device /dev/ttyUSB0:/dev/ttyUSB0 --device /dev/ttyUSB1:/dev/ttyUSB1 -v /dev:/dev --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw" xinsonglin/jetson_nav

FROM arm64v8/ros:humble

# install ros2 nav2
RUN apt update && \
    apt install software-properties-common python3-pip vim zsh -y && \
    apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-turtlebot3* -y

# make nav2 workspace
RUN mkdir -p /ws_nav2/src

# install zlac python
RUN cd /ws_nav2/src && \
    git clone --recursive https://github.com/Lexseal/zlac8015d_ros.git && \
    cd /ws_nav2/src/zlac8015d_ros/ZLAC8015D_python && \
    pip3 install -e .

# clone rplidar ros2
RUN cd /ws_nav2/src && \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# clone nav2 stack
RUN cd /ws_nav2/src && \
    git clone -b empty_belly https://github.com/Lexseal/irobot_create_nav2.git

# build nav2 workspace
RUN ["/bin/bash", "-c", "cd /ws_nav2 && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"]

# source nav2 workspace
RUN echo "source /ws_nav2/install/setup.bash" >> ~/.bashrc && \
    echo "source /ws_nav2/install/setup.zsh" >> ~/.zshrc

# install tmux for multi-terminal
RUN apt install tmux -y && \
    echo "set -g mouse on" >> ~/.tmux.conf

# install realsense stuff
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u -y && \
    apt install librealsense2-utils librealsense2-dev -y && \
    apt install ros-humble-realsense2-* -y
