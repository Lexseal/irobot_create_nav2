# Navigation Stack

## Overview

This package enables the irobot create or the empty belly robot to navigate around using ros2 nav2. In addition, it includes a dockerfile to build and run the entire stack on a jetson (can be easily ported to run on non-arm architectures).

## Installation

Install the dependencies.

```
apt update && \
apt install python3-pip vim zsh -y && \
apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-turtlebot3* -y
```

Then,

### For Empty Belly

1. You will need to get the driver code that enables moving the wheels and sending odometry data.

    ```
    mkdir -p ~/ws_nav2/src && \
    git clone --recursive https://github.com/Lexseal/zlac8015d_ros.git && \
    cd ~/ws_nav2/src/zlac8015d_ros/ZLAC8015D_python && \
    pip3 install -e .
    ```

    The above will install the underlying python communication library for ZLAC8015D, which is the motor driver for the empty belly robot.

2. You will need to clone the lidar package.

    ```
    cd ~/ws_nav2/src && \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
    ```

3. You will obviously also need this very repository.

    ```
    cd ~/ws_nav2/src && \
    git clone -b empty_belly https://github.com/Lexseal/irobot_create_nav2.git
    ```

4. Build the workspace.

    ```
    cd ~/ws_nav2 && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install
    ```

### For iRobot Create

1. Follow https://iroboteducation.github.io/create3_docs/ to get the robot running.

2. You will need to do step 2-4 from the above. Note that when cloning `irobot_create_nav2` you should clone the `master` branch.

## Running

There is a tmux script called `launch.sh` that should bring up everything. Please change the port number in the script as needed.

If you are using docker on jetson, you can pull `xinsonglin/jetson_nav:latest` and start the container with

```
docker run -it --network host --device /dev/ttyUSB0:/dev/ttyUSB0 --device /dev/ttyUSB1:/dev/ttyUSB1 -v /dev:/dev --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw" xinsonglin/jetson_nav:latest`
```

The cgroup rules are needed for realsense camera to work.

## Additional Notes

If you are using L515, please see the dockerfile on how to install proper version of the realsense library as it is discontinued.

There is also a script to convert depth image into 2d lidar scan for either obstacle avoidance or SLAM. However, the realsense cameras have a high minimum range, so it is not very useful for obstacle avoidance.
