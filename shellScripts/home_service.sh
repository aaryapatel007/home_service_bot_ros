#!/bin/sh
xterm -e " cd catkin_ws; source devel/setup.bash; roslaunch my_robot world.launch " &
sleep 5
xterm -e " cd catkin_ws; source devel/setup.bash; roslaunch my_robot amcl.launch " &
sleep 10
xterm -e " cd catkin_ws; source devel/setup.bash; roslaunch add_markers add_markers.launch " &
sleep 5
xterm -e " cd catkin_ws; source devel/setup.bash; roslaunch pick_objects pick_objects.launch "