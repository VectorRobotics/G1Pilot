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
    ld.add_action(DeclareLaunchArgument('goal_pose_topic', default_value='goal_pose'))
    ld.add_action(DeclareLaunchArgument('feedback_topic', default_value='feedback'))
    ld.add_action(DeclareLaunchArgument('position_control_topic', default_value='position_control'))
    ld.add_action(DeclareLaunchArgument('traj_topic', default_value='traj'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_path, 'launch', 'display.launch.py']),
        condition=IfCondition(LaunchConfiguration('PublishTF'))
    ))

    ld.add_action(Node(
        package='g1_pilot',
        executable='visual_servo',
        name='visual_servo',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'position_control_topic': LaunchConfiguration('position_control_topic'),
            'traj_topic': LaunchConfiguration('traj_topic'),
        }],
    ))
    return ld