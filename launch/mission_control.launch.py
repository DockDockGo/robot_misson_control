import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    single_launch = "True"

    robot1_namespace = LaunchConfiguration("robot1_namespace", default="robot1")
    robot2_namespace = LaunchConfiguration("robot2_namespace", default="robot2")
    robot_namespace = LaunchConfiguration("robot_namespace", default="robot1")

    params = os.path.join(
        get_package_share_directory("robot_mission_control"), "config", "params.yaml"
    )

    # Launch the State Machine node with arguments
    mission_control_node = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression(["not ", single_launch])),
                package="robot_mission_control",
                executable="robot_mission_control_node",
                name="robot_mission_control_node",
                output="screen",
                parameters=[
                    {"robot1_namespace_param": robot1_namespace},
                    {"robot2_namespace_param": robot2_namespace},
                    {"mission_params": params},
                ],
            ),
            Node(
                condition=IfCondition(PythonExpression([single_launch])),
                package="robot_mission_control",
                executable="robot_mission_control_node_single",
                name="robot_mission_control_node_single",
                output="screen",
                parameters=[
                    {"robot_namespace_param": robot_namespace},
                    {"mission_params": params},
                ],
            ),
        ]
    )

    ld.add_action(mission_control_node)

    return ld
