from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---- path ai param YAML (pacchetto di esempio: esn_tutorial)
    pkg_share = get_package_share_directory('esn_tutorial')
    cfg_dir   = os.path.join(pkg_share, 'config')

    limit_switch_yaml = os.path.join(cfg_dir, 'limit_switch_params.yaml')
    vision_yaml       = os.path.join(cfg_dir, 'vision_system_params.yaml')
    robot_action_yaml = os.path.join(cfg_dir, 'robot_action_params.yaml')
    orechestrator_yaml = os.path.join(cfg_dir, 'orchestrator_params.yaml')

    # ---- nodi

    # 1) Nodo finecorsa / limit switch
    limit_switch_node = Node(
        package='esn_tutorial',
        executable='limit_switch_node',
        name='limit_switch_node',
        parameters=[limit_switch_yaml],
        output='screen'
    )

    # 2) Nodo service del sistema di visione
    vision_srv_node = Node(
        package='esn_tutorial',
        executable='vision_srv_server_node',
        name='vision_srv_server_node',
        parameters=[vision_yaml],
        output='screen'
    )

    # 3) Nodo applicazione (orchestratore)
    app_node = Node(
        package='esn_tutorial',
        executable='orchestrator_node',
        name='orchestrator_node',
        parameters=[orechestrator_yaml],
        output='screen'
    )

    # 4) Nodo action server pianificazione/controllo robot
    robot_action_server = Node(
        package='esn_tutorial',
        executable='robot_action_server_node',
        name='robot_action_node',
        parameters=[robot_action_yaml],
        output='screen'
    )

    return LaunchDescription([
        limit_switch_node,
        vision_srv_node,
        app_node,
        robot_action_server,
    ])
