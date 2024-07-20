from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python import get_package_share_path
# from launch_ros.parameter_descriptions import ParameterValue

import os
def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('nemo_master'),'description','robot.urdf.xacro')
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description":Command(['xacro ',urdf_path, " is_sim:=False"])}
        ]
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description":Command(['xacro ',urdf_path, " is_sim:=False"]),
             "use_sim_time":False},
            os.path.join(
                get_package_share_path("nemo_controller"),"config",
                "nemo_controller.yaml"
            )
        ]
    )

    return LaunchDescription([
robot_state_pub,
controller_manager
    ])