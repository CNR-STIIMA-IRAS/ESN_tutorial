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
            'use_gui',
            default_value='true',
            description='Se true usa joint_state_publisher_gui, altrimenti joint_state_publisher'
        ),
  ]

  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):

  robot_description_path = LaunchConfiguration('model').perform(context)
  robot_description_args = {
    # 'robot_ip' : LaunchConfiguration('robot_ip').perform(context),
    # 'use_fake_hardware' : LaunchConfiguration('use_fake_hardware').perform(context),
    # 'prefix' : f'{LaunchConfiguration("prefix").perform(context)}/',
  }

  robot_description = xacro.process(robot_description_path, mappings=robot_description_args)

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description' : ParameterValue(value=robot_description, value_type=str)}]
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

  joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen',
    condition=IfCondition(LaunchConfiguration('use_gui'))
  )

  

  return [
    robot_state_publisher,
    rviz_node,
    joint_state_publisher_gui_node
  ]