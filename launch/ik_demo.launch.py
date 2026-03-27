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
    ld.add_action(DeclareLaunchArgument('position_control_topic', default_value='position_control'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_path, 'launch', 'display.launch.py']),
        condition=IfCondition(LaunchConfiguration('PublishTF'))
    ))

    ld.add_action(Node(
        package='g1_pilot',
        executable='ik_joint_state_publisher',
        name='ik_joint_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'position_control_topic': LaunchConfiguration('position_control_topic'),
        }],
    ))
    return ld