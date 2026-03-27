from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition



def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable(
        'LIBRARY_PATH',
        ['/opt/openrobots/lib:', EnvironmentVariable('LIBRARY_PATH', default_value='')]
    ))
    ld.add_action(SetEnvironmentVariable(
        'PATH',
        ['/opt/openrobots/bin:', EnvironmentVariable('PATH', default_value='')]
    ))
    ld.add_action(SetEnvironmentVariable(
        'PKG_CONFIG_PATH',
        ['/opt/openrobots/lib/pkgconfig:', EnvironmentVariable('PKG_CONFIG_PATH', default_value='')]
    ))
    ld.add_action(SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        ['/opt/openrobots/lib:', EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]
    ))
    ld.add_action(SetEnvironmentVariable(
        'PYTHONPATH',
        ['/opt/openrobots/lib/python3.10/site-packages:', EnvironmentVariable('PYTHONPATH', default_value='')]
    ))
    ld.add_action(SetEnvironmentVariable(
        'CMAKE_PREFIX_PATH',
        ['/opt/openrobots:', EnvironmentVariable('CMAKE_PREFIX_PATH', default_value='')]
    ))

    package_name = 'g1_pilot'
    package_path = FindPackageShare(package_name)

    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument(
        'PublishTF', default_value='false',
        description='Whether to include display.launch.py for TF publishing'
    ))
    ld.add_action(DeclareLaunchArgument('goal_pose_topic', default_value='arm/goal_pose'))
    ld.add_action(DeclareLaunchArgument('feedback_topic', default_value='feedback'))
    ld.add_action(DeclareLaunchArgument('position_control_topic', default_value='position_control'))
    ld.add_action(DeclareLaunchArgument('traj_topic', default_value='traj'))
    ld.add_action(DeclareLaunchArgument('goal_cooldown', default_value='1.0'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_path, 'launch', 'display.launch.py']),
        condition=IfCondition(LaunchConfiguration('PublishTF'))
    ))

    ld.add_action(Node(
        package='g1_pilot',
        executable='visual_servo_tf',
        name='visual_servo_tf',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
            'feedback_topic': LaunchConfiguration('feedback_topic'),
            'position_control_topic': LaunchConfiguration('position_control_topic'),
            'traj_topic': LaunchConfiguration('traj_topic'),
            'goal_cooldown': LaunchConfiguration('goal_cooldown'),
        }],
        remappings=[
            ('/track3d/selected_normal_pose_filtered_upright', PythonExpression(["'", LaunchConfiguration('goal_pose_topic'), "/right'"]))
        ]
    ))
    return ld