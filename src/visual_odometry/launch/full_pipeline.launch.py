from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    """
    airsim_node = Node(
        package='airsim_interface',
        executable='airsim_interface',
        name='airsim_interface',
    )
    """

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

    # remap topic /camera/aligned_depth_to_color/image_raw to

    visual_odom_node = Node(
        package='visual_odometry',
        executable='visual_odometry',
        name='visual_odometry',
    )

    visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('visualizer'), 'visualize.launch.py'])
        ),
        launch_arguments={
            'rviz_config': PathJoinSubstitution([FindPackageShare('visualizer'), 'visualize.rviz']),
        }.items(),
    )

    return LaunchDescription([
        # airsim_node,
        visual_odom_node,
        visualizer_launch,
        launch_realsense,
    ])