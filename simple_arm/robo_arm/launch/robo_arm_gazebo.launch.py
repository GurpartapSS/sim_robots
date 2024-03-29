from launch import LaunchDescription, launch_description_sources, actions
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python import get_package_share_path
import os

def generate_launch_description():
    ld = LaunchDescription()
    urdf_path = os.path.join(get_package_share_path('robo_arm'),'description','robot_arm.urdf.xacro')
    rviz_path = os.path.join(get_package_share_path('robo_arm'),'rviz','robo_arm.rviz')
    # world_path = os.path.join(get_package_share_path('my_robot_description'),'world','my_robot.world')

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description":Command(['xacro ',urdf_path])}
        ]
    )

    launch_gazeo = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join('/opt/ros/humble/share/gazebo_ros/launch/', 'gazebo.launch.py')))
            # launch_arguments={'world': world_path}.items())

    spawmer_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', '/robot_description','-entity', 'Maya']
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_path]
    )

    ld.add_action(robot_state_pub)
    ld.add_action(launch_gazeo)
    ld.add_action(spawmer_node)
    # ld.add_action(rviz_node)
    return ld