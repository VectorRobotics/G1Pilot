from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'g1_pilot'

    description_pkg_name = 'g1_description'
    description_pkg_path = FindPackageShare(description_pkg_name)

    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument(
        'PublishTF', default_value='false',
        description='Whether to include display.launch.py for TF publishing'
    ))
    ld.add_action(DeclareLaunchArgument('joint_trajectory_topic', default_value='joint_trajectory'))
    ld.add_action(DeclareLaunchArgument('traj_topic', default_value='traj'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([description_pkg_path, 'launch', 'display.launch.py']),
        condition=IfCondition(LaunchConfiguration('PublishTF'))
    ))

    ld.add_action(Node(
        package=package_name,
        executable='joint_traj_publisher',
        name='joint_traj_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'joint_trajectory_topic': LaunchConfiguration('joint_trajectory_topic'),
            'traj_topic': LaunchConfiguration('traj_topic'),
        }],
    ))
    return ld
