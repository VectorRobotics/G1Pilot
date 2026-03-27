from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource



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
    perception_path = FindPackageShare('hand_eye_calibration')

    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument(
        'PublishTF', default_value='true',
        description='Whether to include display.launch.py for TF publishing'
    ))
    ld.add_action(DeclareLaunchArgument(
        'marker_size', default_value='0.1058',
        description='Physical size of the ArUco marker in meters',
    ))
    ld.add_action(DeclareLaunchArgument('goal_pose_topic', default_value='goal_pose'))
    ld.add_action(DeclareLaunchArgument('feedback_topic', default_value='feedback'))
    ld.add_action(DeclareLaunchArgument('position_control_topic', default_value='position_control'))
    ld.add_action(DeclareLaunchArgument('traj_topic', default_value='traj'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_path, 'launch', 'display.launch.py']),
        condition=IfCondition(LaunchConfiguration('PublishTF'))
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'depth_module.profile': '1280x960x30',
            'rgb_camera.profile': '1280x960x30',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
        }.items()
    ))

    # --- ArUco detector ---
    ld.add_action(Node(
        package='hand_eye_calibration',
        executable='aruco_pose_detector',
        name='aruco_pose_detector',
        output='screen',
        parameters=[
            PathJoinSubstitution([perception_path, 'config', 'aruco_params.yaml']),
            {'marker_size': LaunchConfiguration('marker_size')},
        ],
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