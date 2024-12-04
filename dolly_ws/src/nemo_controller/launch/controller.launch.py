
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_simple_conroller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="true"
    )

    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    wheel_vel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "nemo_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions=[
            Node(
                package="nemo_controller",
                executable="simple_controller"
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
        ]
    )

    return LaunchDescription([
        use_simple_conroller_arg,
        joint_state_broad,
        wheel_vel_controller,
        simple_controller
    ])