# color_control
ROS node where a turtlebot will drive to different color spheres in a Gazebo environment   
Created by Mitchell Scott and Oscar de Haro through Washington State University with assistance from Matt Taylor  


## About
This is a basic ROS node where a turtlebot will drive around a gazebo enviornment to different color spheres using a color mask and a PD controller  

Requirments: ROS, gazebo, and the ROS turtlebot_gazebo package  
## Installation

$cd <catkin_ws>/src   
$git clone https://github.com/miscott/color_control.git  
$cd <catkin_ws>   
$catkin_make  

## To run
$roscore  
On new terminal: $roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<path_to_catkin_ws>/src/color_control/src/full_world  
On new terminal: $rosrun color_control drive_to_spheres.py  

The turtlebot will now drive to different color spheres unit the user terminates the drive_to_spheres.py node.  

## Trouble Shooting:

If any problems occur, make sure enviornment is properly source ($source ~/catkin_ws/devel/setup.bash)  
Verify that turtlebot_gazebo package was properly installed by running $roslaunch turtlebot_gazebo turtlebot_world.launch  
Verify that catkin_make finished properly  

