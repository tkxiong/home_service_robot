
#!/bin/sh
xterm  -e  " source /home/$USER/hsr_catkin_ws/devel/setup.bash; roslaunch home_service_robot turtlebot_world.launch"  &
sleep 5
xterm  -e  " source /home/$USER/hsr_catkin_ws/devel/setup.bash; roslaunch home_service_robot amcl_demo.launch" &
sleep 2
xterm  -e  " source /home/$USER/hsr_catkin_ws/devel/setup.bash; roslaunch home_service_robot view_navigation.launch" &
xterm  -e  " source /home/$USER/hsr_catkin_ws/devel/setup.bash; rosrun add_markers add_markers_node " &
sleep 5
xterm  -e "source /home/$USER/hsr_catkin_ws/devel/setup.bash; rostopic pub /pickup_goal pick_objects/goal \"first_x: {data: 4.6}
first_y: {data: 4.8}
first_w: {data: 1.0}
second_x: {data: 4.2}
second_y: {data: 0.0}
second_w: {data: 1.0}
check_pos: {data: false}\" --once "