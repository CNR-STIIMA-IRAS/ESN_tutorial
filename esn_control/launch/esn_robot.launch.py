from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import xacro
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
  launch_args = [
        DeclareLaunchArgument(
            'model',
            description='Percorso a un file URDF o XACRO da visualizzare',
            default_value=PathJoinSubstitution([FindPackageShare('esn_description'), 'urdf', 'esn_robot.urdf.xacro'])
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare('esn_description'), 'rviz', 'config.rviz']),
            description='(Opzionale) Percorso a un file RViz (.rviz)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usa /clock dalla simulazione'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='(Opzionale) Prefisso da aggiungere al robot'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Se true usa l\'hardware in simulazione, altrimenti l\'hardware reale'
        ),
        DeclareLaunchArgument(
            'ros2_control_config',
            default_value=PathJoinSubstitution([FindPackageShare('esn_control'), 'config', 'ros2_controllers.yaml']),
            description='Percorso al file di configurazione dei controller'
        ),
  ]

  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):

  robot_description_path = LaunchConfiguration('model').perform(context)
  robot_description_args = {
    # 'robot_ip' : LaunchConfiguration('robot_ip').perform(context),
    'use_fake_hardware' : LaunchConfiguration('use_fake_hardware').perform(context),
    'prefix' : f'{LaunchConfiguration("prefix").perform(context)}',
  }

  robot_description = xacro.process(robot_description_path, mappings=robot_description_args)
  robot_description_param = {'robot_description' : ParameterValue(value=robot_description, value_type=str)}
  
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[robot_description_param]
  )

  rviz_config = LaunchConfiguration('rviz_config').perform(context)
  rviz_args = ['-d', rviz_config] if rviz_config else []

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=rviz_args
  )

  # ROS2 CONTROL NODES
  ros2_control_config = LaunchConfiguration('ros2_control_config')
  control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    output="screen",
    parameters=[robot_description_param, ros2_control_config],
    arguments=["--ros-args", "--log-level", "info"],
  )

  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"],
  )

  diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_drive_controller",
                "--controller-manager",
                "/controller_manager"],
  )

  state_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["state_controller",
                "--controller-manager",
                "/controller_manager"],
    condition=UnlessCondition(LaunchConfiguration('use_fake_hardware'))
  )
  return [
    robot_state_publisher,
    rviz_node,
    control_node,
    joint_state_broadcaster_spawner,
    diff_drive_spawner, 
    state_controller
  ]