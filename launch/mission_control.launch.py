import launch_ros.actions
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    #### Define the params ####
    #! Defined robot0 and robot1 only for sim
    robot1_namespace = LaunchConfiguration("robot1_namespace", default="robot1")
    robot2_namespace = LaunchConfiguration("robot2_namespace", default="robot2")

    params = os.path.join(
        get_package_share_directory('robot_mission_control'),
        'config',
        'params.yaml'
        )

    # Launch the State Machine node with arguments
    MissonControl = launch_ros.actions.Node(
        package="robot_mission_control",
        executable="robot_mission_control_node",  # NOTE: Executable name defined in setup.py
        name="robot_mission_control_node",
        output="screen",
        parameters=[
            {"robot1_namespace_param": robot1_namespace},
            {"robot2_namespace_param": robot2_namespace},
            {'mission_params': params}
        ],
    )

    ld.add_action(MissonControl)

    return ld
