RBE3002 Group DMN

Dominic Cupo - Domo929

Nick Hollan - nchollan

Michael Hopkins - themhoppy

For our lab 3

https://github.com/Domo929/RBE3002-Group



In order to run this code!!!!

----------------
Before anything else
----------------
1) Copy the setup contents of NewCMake.txt into your Cmake file
2) Setup networking between the turtlebot and your computer
3) Test connection

----------------
Turtlebot Setup
----------------
1)roslaunch turtlebot_bringup minimal.launch
2)roslaunch turtlebot_navigation gmapping_demo.launch

----------------
Computer Setup
----------------
1)Make sure your workspace is sourced properly
2)rosrun rqt_reconfigure rqt_reconfigure
	Once the window opens:
	-Navigate to the move_base_node section
	-Select TrajectoryPlannerROS
	-In there change the heading_scoring parameter to true
3)roslaunch turtlebot_rviz_launchers view_navigation.launch
4)rosrun DMN_lab3 exploration.py


----------------
Final Step
----------------
WATCH IN AWE