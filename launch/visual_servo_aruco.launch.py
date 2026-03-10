from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    ld = LaunchDescription()

    package_path = FindPackageShare('g1_pilot')
    default_model_path = PathJoinSubstitution([package_path, 'assets', 'g1', 'g1_29dof_with_hand_rev_1_0_ros.urdf'])
    default_rviz_config_path = PathJoinSubstitution([package_path, 'rviz', 'path.rviz'])

    perception_path = FindPackageShare('hand_eye_calibration')

    ld.add_action(DeclareLaunchArgument(
        'marker_size', default_value='0.1058',
        description='Physical size of the ArUco marker in meters',
    ))

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='jsp_gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    robot_description_content = ParameterValue(Command(['xacro ', default_model_path]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    ld.add_action(robot_state_publisher_node)

    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
        parameters=[{
            'source_list': ['position_control'],
        }]
    ))
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    ))

    realsense_launch = IncludeLaunchDescription(
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
    )
    ld.add_action(realsense_launch)

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'd435_link',
            '--child-frame-id', 'camera_link',
        ],
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
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    ld.add_action(Node(
        package='g1_pilot',
        executable='visual_servo',
        name='visual_servo',
        output='screen',
    ))
    return ld