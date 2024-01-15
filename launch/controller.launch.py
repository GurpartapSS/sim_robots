from launch import LaunchDescription, launch_description_sources, actions
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python import get_package_share_path
import os

def generate_launch_description():
    ld = LaunchDescription()
    urdf_path = os.path.join(get_package_share_path('robo_arm'),'description','robot_arm.urdf.xacro')


    robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description":Command(['xacro ',urdf_path])}
            ]
        )

    joint_state_brod_=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"
                   ]
    )
    arm_controller_=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager",
                   "/controller_manager"
                   ]
    )
    gripper_controller_=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "--controller-manager",
                   "/controller_manager"
                   ]
    )


    ld.add_action(robot_state_pub)
    ld.add_action(joint_state_brod_)
    ld.add_action(arm_controller_)
    ld.add_action(gripper_controller_)
    return ld