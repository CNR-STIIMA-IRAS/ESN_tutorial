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


    vision_system_node_cmd = Node(
        package='esn_tutorial_py',
        executable='vision_system_node',
        name='vision_system_node',
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


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(vision_config_path_cmd)
    ld.add_action(limit_switch_config_path_cmd)

    ld.add_action(vision_system_node_cmd)
    ld.add_action(limit_switch_node_cmd)

    return ld