# mobile_robotics

Clone this repository and follow the given command
'''bash
"catkin_make"
'''

For Question 2, go to the turtlebot3 repository given in the assignment then follow the following steps
'''bash
"source ~/catkin_ws/devel/setup.bash"
"export TURTLEBOT3_MODEL=burger"
"roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch"
'''

Now go to mobile_robotics repository and follow following commands
'''bash
"source devel/setup.bash"
"./Q2.sh"
'''

The turtlebot will start moving in a circle of radius approximately 1.5 m in counter clockwise direction.

For Question 3, go to the turtlebot3 repository given in the assignment then follow the following steps
'''bash
"source ~/catkin_ws/devel/setup.bash"
"roslaunch turtlebot3_mr turtlebot3_lab2.launch"
"roslaunch turtlebot3_mr apriltag_gazebo.launch"
'''

Now go to mobile_robotics repository and follow following commands
'''bash
"source devel/setup.bash"
"rosrun mobile_robotics april_tag_trajectory.py"
'''
