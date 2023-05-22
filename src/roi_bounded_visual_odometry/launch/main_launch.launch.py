from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch/rs_launch.py'
                ])
            ]),
        launch_arguments=[('depth_module.profile', '1280x720x30'),
                          ('pointcloud.enable', 'true'),
                          ('align_depth.enable', 'true')]
    )

    visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('visualizer'), 'visualize.launch.py'])
        ),
        launch_arguments={
            'rviz_config': PathJoinSubstitution([FindPackageShare('visualizer'), 'visualize.rviz']),
        }.items(),
    )

    visual_odom_node = Node(
        package='roi_bounded_visual_odometry',
        executable='visual_odometry',
        name='visual_odometry',
    )

    return LaunchDescription([
        visual_odom_node,
        launch_realsense,
        visualizer_launch
    ])