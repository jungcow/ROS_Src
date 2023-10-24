#! /bin/bash

iterm-terminal 'mamba activate ros_env && roscore'
sleep 5
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn controller'
sleep 5
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 1 __name:=edge_computer1'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 2 __name:=edge_computer2'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 3 __name:=edge_computer3'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 4 __name:=edge_computer4'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 5 __name:=edge_computer5'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 6 __name:=edge_computer6'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 7 __name:=edge_computer7'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 8 __name:=edge_computer8'
sleep 1
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn edge_computer 9 __name:=edge_computer9'
sleep 10
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn vehicle 1 1 6 0 __name:=vehicle1'
iterm-terminal 'mamba activate ros_env && \
	source ~/catkin_ws/devel/setup.sh && \
	rosrun sdn vehicle 2 6 7 0 __name:=vehicle2'

