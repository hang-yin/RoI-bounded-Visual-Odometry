from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration, \
                                 TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    share_folder = get_package_share_path('visualizer')
    rviz_config_path = share_folder / 'visualize.rviz'

    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(rviz_config_path),
        description='Path to the RViz config file to use')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        rviz_node,
    ])