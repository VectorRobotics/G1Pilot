from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command



def generate_launch_description():
    ld = LaunchDescription()

    package_path = FindPackageShare('g1_description')
    default_model_path = PathJoinSubstitution([package_path, 'assets', 'g1', 'g1_29dof_with_hand_rev_1_0_ros.urdf'])
    ctrl_viz_model_path = PathJoinSubstitution([package_path, 'assets', 'g1', 'g1_29dof_with_hand_rev_1_0_ros_ctrl_viz.urdf'])
    default_rviz_config_path = PathJoinSubstitution([package_path, 'rviz', 'manipulation.rviz'])

    # These parameters are maintained for backwards compatibility
    rviz_arg = DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    robot_description_content = ParameterValue(Command(['xacro ', default_model_path]), value_type=str)
    ctrl_viz_description_content = ParameterValue(Command(['xacro ', ctrl_viz_model_path]), value_type=str)

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
        }],
        remappings=[('joint_states', 'js_feedback')]
    ))

    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/feedback'],
        }],
        remappings=[('joint_states', 'js_feedback')]
    ))
    
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='control_viz',
        parameters=[{
            'robot_description': ctrl_viz_description_content,
            'frame_prefix': 'control/'
        }],
        remappings=[('joint_states', 'js_control')]
    ))

    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='control_viz',
        parameters=[{
            'source_list': ['/position_control'],
        }],
        remappings=[('joint_states', 'js_control')]
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='control_feedback_bridge_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'control/base_link']
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    return ld