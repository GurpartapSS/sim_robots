Install ROS Humble
Install colcon, build a skeleton ws 
Install joint_state_publisher_gui to get the wheel joints pubished to Rviz correctly use_sim_time:=true
Build the robot in URDF and run it in RVIZ with fixed frame as base_link
Install Gazebo -- make changes to the world as you like and then use world:="world_name"
use teleop_twist keyboard or controller to run the robot around and see the odom, baselink and wheel transforms
Install slam_toolbox use online_async with sim_time_true to publish the map, keep the fixed_frame to map
change mapper*.yaml file to load the start mapping or load an old map and specify in params_file:= if the file is changed at a different location 


road map --

add camera 
change the rplidar scaning from a certain range 
check gazebo arm
add gazzebo arm
