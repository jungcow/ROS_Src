#! /bin/bash

iterm-terminal 'mamba activate ros_env && roscore'
sleep 5
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn controller'
sleep 5
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 1'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 2'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 3'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 4'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 5'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 6'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 7'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 8'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 9'
sleep 5
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn vehicle 1 1 6 0'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn vehicle 2 6 7 0'

