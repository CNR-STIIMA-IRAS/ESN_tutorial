from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('esn_tutorial_py')

    vision_config_path_cmd = DeclareLaunchArgument(
        'vision_config_path',
        default_value=pkg_dir + '/config/vision_system_params.yaml',
        description='Full path to the config file of the vision system node')
    
    limit_switch_config_path_cmd = DeclareLaunchArgument(
        'limit_switch_config_path',
        default_value=pkg_dir + '/config/limit_switch_params.yaml',
        description='Full path to the config file of the limit switch node')

    robot_action_config_path_cmd = DeclareLaunchArgument(
        'robot_action_config_path',
        default_value=pkg_dir + '/config/robot_action_params.yaml',
        description='Full path to the config file of the robot action node')

    orchestrator_config_path_cmd = DeclareLaunchArgument(
        'orchestrator_config_path',
        default_value=pkg_dir + '/config/orchestrator_params.yaml',
        description='Full path to the config file of the orchestrator node')
    
    vision_server_node_cmd = Node(
        package='esn_tutorial_py',
        executable='vision_server_node',
        name='vision_server_node',
        output='screen',
        parameters=[
            LaunchConfiguration('vision_config_path')
    ])
    
    limit_switch_node_cmd = Node(
        package='esn_tutorial_py',
        executable='limit_switch_node',
        name='limit_switch_node',
        output='screen',
        parameters=[
            LaunchConfiguration('limit_switch_config_path')
        ])
    
    robot_action_node_cmd = Node(
        package='esn_tutorial_py',
        executable='robot_action_server_node',
        name='robot_action_server_node',
        output='screen',
        parameters=[
            LaunchConfiguration('robot_action_config_path')
        ])
    
    orchestrator_node_cmd = Node(
        package='esn_tutorial_py',
        executable='orchestrator_node',
        name='orchestrator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('orchestrator_config_path')
        ])
    


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(vision_config_path_cmd)
    ld.add_action(limit_switch_config_path_cmd)
    ld.add_action(robot_action_config_path_cmd)
    ld.add_action(orchestrator_config_path_cmd)

    ld.add_action(vision_server_node_cmd)
    ld.add_action(limit_switch_node_cmd)
    ld.add_action(robot_action_node_cmd)
    ld.add_action(orchestrator_node_cmd)

    return ld