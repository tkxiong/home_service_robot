
#!/bin/sh
xterm  -e  " source /home/$USER/hsr_catkin_ws/devel/setup.bash; roslaunch home_service_robot turtlebot_world.launch"  &
sleep 5
xterm  -e  " source /home/$USER/hsr_catkin_ws/devel/setup.bash; roslaunch home_service_robot amcl_demo.launch" &
sleep 2
xterm  -e  " source /home/$USER/hsr_catkin_ws/devel/setup.bash; roslaunch home_service_robot view_navigation.launch"
