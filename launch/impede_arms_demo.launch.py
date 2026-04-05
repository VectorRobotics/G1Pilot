from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'g1_pilot'
    package_path = FindPackageShare(package_name)

    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument(
        'PublishTF', default_value='true',
        description='Whether to include display.launch.py for TF publishing'
    ))
    ld.add_action(DeclareLaunchArgument('feedback_topic', default_value='feedback'))
    ld.add_action(DeclareLaunchArgument('goal_pose_topic', default_value='goal_pose'))
    ld.add_action(DeclareLaunchArgument('effort_control_topic', default_value='effort_control'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_path, 'launch', 'display.launch.py']),
        condition=IfCondition(LaunchConfiguration('PublishTF'))
    ))

    ld.add_action(Node(
        package='g1_pilot',
        executable='impede_arms',
        name='impede_arms',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
            'effort_control_topic': LaunchConfiguration('effort_control_topic'),
        }],
    ))
    return ld