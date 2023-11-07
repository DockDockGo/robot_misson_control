import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ld = LaunchDescription()

    #### Define the params ####
    #! Defined robot0 and robot1 only for sim
    # robot1_namespace = LaunchConfiguration('robot1_namespace', default="")

    # Launch the State Machine node with arguments
    MissonControl = launch_ros.actions.Node(
        package='robot_mission_control',
        executable='robot_mission_control_node_single', # NOTE: Executable name defined in setup.py
        name='robot_mission_control_node_single',
        output='screen',
        parameters=[{'robot2_namespace_param': "robot2"}]
    )

    ld.add_action(MissonControl)

    return ld