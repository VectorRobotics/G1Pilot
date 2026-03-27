from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition



def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable(
        'LIBRARY_PATH',
        ['/opt/openrobots/lib:', EnvironmentVariable('LIBRARY_PATH', default_value='')]
    ))

    package_name = 'g1_pilot'
    package_path = FindPackageShare(package_name)

    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument(
        'PublishTF', default_value='true',
        description='Whether to include display.launch.py for TF publishing'
    ))
    ld.add_action(DeclareLaunchArgument('feedback_topic', default_value='feedback'))
    ld.add_action(DeclareLaunchArgument('effort_control_topic', default_value='effort_control'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_path, 'launch', 'display.launch.py']),
        condition=IfCondition(LaunchConfiguration('PublishTF'))
    ))

    ld.add_action(Node(
        package='g1_pilot',
        executable='grav_ff',
        name='grav_ff',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'effort_control_topic': LaunchConfiguration('effort_control_topic'),
        }],
    ))
    return ld