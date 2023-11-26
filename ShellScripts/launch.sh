#! /bin/bash

gnome-terminal -e "roscore" &
sleep 5
gnome-terminal -e "rosrun sdn application" &
sleep 5
gnome-terminal -e "rosrun sdn controller" &
sleep 5
gnome-terminal -e "rosrun sdn edge_computer 1 __name:=edge_computer1" &
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 2 __name:=edge_computer2" &
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 3 __name:=edge_computer3" & 
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 4 __name:=edge_computer4" &
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 5 __name:=edge_computer5" &
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 6 __name:=edge_computer6" &
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 7 __name:=edge_computer7" & 
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 8 __name:=edge_computer8" & 
sleep 1
gnome-terminal -e "rosrun sdn edge_computer 9 __name:=edge_computer9" & 
sleep 5
gnome-terminal -e "rosrun sdn vehicle 1 1 9 0 __name:=vehicle1" & 
gnome-terminal -e "rosrun sdn vehicle 2 9 4 0 __name:=vehicle2" & 
gnome-terminal -e "rosrun sdn vehicle 3 8 3 0 __name:=vehicle3" &
gnome-terminal -e "rosrun sdn vehicle 4 7 6 0 __name:=vehicle4" &
gnome-terminal -e "rosrun sdn vehicle 5 1 6 0 __name:=vehicle5" &
gnome-terminal -e "rqt_graph" &

