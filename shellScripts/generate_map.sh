#!/bin/sh
xterm -e " cd catkin_ws; source devel/setup.bash; roslaunch my_robot world.launch " &
sleep 5
xterm -e " cd catkin_ws; source devel/setup.bash; roslaunch my_robot mapping.launch " 