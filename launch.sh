#!/bin/bash

# Start a new tmux session named "mysession"
tmux new-session -d -s mysession

# Split the window into two panes horizontally
tmux split-window -h -t mysession

# Select the left pane
tmux select-pane -t mysession:0.0

# Split the left pane vertically
tmux split-window -v -t mysession

# Select the right pane
tmux select-pane -t mysession:0.2

# Split the right pane vertically
tmux split-window -v -t mysession

# Send commands to each pane
tmux send-keys -t mysession:0.0 'ros2 run zlac8015d_ros driver /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0019X5N-if00-port0' C-m
tmux send-keys -t mysession:0.1 'ros2 launch irobot_create_nav2 
navigation2.launch.py serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0' C-m
tmux send-keys -t mysession:0.2 'ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=960x540x6 depth_module.profile:=640x480x30 enable_sync:=True align_depth.enable:=True' C-m
tmux send-keys -t mysession:0.3 '' C-m

# Attach to the tmux session to view the panes
tmux attach-session -t mysession