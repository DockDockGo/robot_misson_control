import os
import launch_ros.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ld = LaunchDescription()

    #### Define the params ####
    #! Defined robot0 and robot1 only for sim
    # robot1_namespace = LaunchConfiguration('robot1_namespace', default="")

    params = os.path.join(
        get_package_share_directory('robot_mission_control'),
        'config',
        'params.yaml'
        )

    # Launch the State Machine node with arguments
    MissonControl = launch_ros.actions.Node(
        package='robot_mission_control',
        executable='robot_mission_control_node_single', # NOTE: Executable name defined in setup.py
        name='robot_mission_control_node_single',
        output='screen',
        parameters=[{'robot2_namespace_param': "robot2"},{'mission_params': params}]
    )

    ld.add_action(MissonControl)

    return ld