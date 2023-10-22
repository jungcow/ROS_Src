#! /bin/bash

iterm-terminal 'mamba activate ros_env && roscore'
sleep 5
iterm-terminal 'mamba activate ros_env && rosrun turtlesim turtlesim_node'
sleep 5
iterm-terminal 'mamba activate ros_env && rosrun turtlesim turtle_teleop_key'

