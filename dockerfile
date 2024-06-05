# docker run -it --network host --device /dev/ttyUSB0:/dev/ttyUSB0 --device /dev/ttyUSB1:/dev/ttyUSB1 -v /dev:/dev --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw" xinsonglin/jetson_nav

FROM arm64v8/ros:humble

# install ros2 nav2
RUN apt update && \
    apt install python3-pip vim zsh -y && \
    apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-turtlebot3* -y

# make nav2 workspace
RUN mkdir -p /ws_nav2/src

# install zlac python
# note that you may or may not need to pip install transforms3d yourself in the running container
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

RUN ["/bin/bash", "-c", "cd /ws_nav2 && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"]

# source nav2 workspace
RUN echo "source /ws_nav2/install/setup.bash" >> ~/.bashrc && \
    echo "source /ws_nav2/install/setup.zsh" >> ~/.zshrc

# install tmux for multi-terminal
RUN apt install tmux -y && \
    echo "set -g mouse on" >> ~/.tmux.conf

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Builder dependencies installation
RUN apt update \
    && apt install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN git clone -b l515-global-clock https://github.com/Lexseal/librealsense.git

# Build and install
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \    
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=false \
    -DCMAKE_BUILD_TYPE=Release ../ \
    && make -j16 all \
    && make install

# Install dep packages for realsense
RUN apt update \
    && apt install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/librealsense/lib' >> ~/.bashrc && \
    echo 'export PATH=$PATH:/opt/librealsense/bin' >> ~/.bashrc

RUN ["/bin/bash", "-c", "cd /ws_nav2/src && \
     git clone -b 4.55.1 https://github.com/IntelRealSense/realsense-ros.git && \
     apt install python3-rosdep -y && \
     rosdep init ; rosdep update"]

RUN ["/bin/bash", "-c", "cd /ws_nav2/ && apt update && \
     rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y"]

# Some environment variables aren't right so this compilation needs to be done in the runnning container for now
# RUN ["/bin/bash", "-c", "cd /ws_nav2/ && \
#      source ~/.bashrc && \
#      source /opt/ros/$ROS_DISTRO/setup.bash && \
#      colcon build --symlink-install"]

# ORBSLAM3 not used
# RUN cd ~ && \
#     git clone https://github.com/Lexseal/ORB_SLAM3.git && \
#     cd ORB_SLAM3 && \
#     ./build.sh

# RUN cd /ws_nav2/src && \
#     git clone https://github.com/Lexseal/ORB_SLAM3_ROS2.git

# RUN ["/bin/bash", "-c", "cd /ws_nav2/ && \
#      source ~/.bashrc && \
#      source /opt/ros/$ROS_DISTRO/setup.bash && \
#      colcon build --symlink-install"]

WORKDIR /
