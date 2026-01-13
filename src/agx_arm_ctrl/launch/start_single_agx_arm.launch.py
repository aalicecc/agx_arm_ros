from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"  

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )

    # Arguments
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the AGX Arm node.'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable the AGX Arm node.'
    )

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='piper',
        description='Type of robotic arm.',
        choices=['piper', 'nero']
    )

    # TODO
    end_effector_type_arg = DeclareLaunchArgument(
        'end_effector_type',
        default_value='test0',
        description='Type of end effector.',
        choices=['test0', 'test1']
    )

    use_upper_ik_arg = DeclareLaunchArgument(
        'use_upper_ik',
        default_value='true',
        description='Whether to use upper IK solver.'
    )

    # Default URDF path will be set based on arm_type in the node
    agx_arm_desc_share = get_package_share_directory('agx_arm_description')
    default_urdf_path = PathJoinSubstitution([
        agx_arm_desc_share,
        LaunchConfiguration("arm_type"),
        "urdf",
        "nero.urdf"
    ])
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
        description='Path to the URDF file for the robot.'
    )

    # Node
    agx_arm_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_single_ctrl',
        name='agx_arm_ctrl_single_node',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'arm_type': LaunchConfiguration('arm_type'),
            'end_effector_type': LaunchConfiguration('end_effector_type'),
            'use_upper_ik': LaunchConfiguration('use_upper_ik'),
            'urdf_path': LaunchConfiguration('urdf_path'),
        }],
        remappings=[
            # feedback topics
            ('/feedback/joint_states', '/feedback/joint_states'),
            ('/feedback/end_pose', '/feedback/end_pose'),

            # control topics
            ('/control/joint_states', '/control/joint_states'),
            ('/control/end_pose', '/control/end_pose'),

            # services
            ('enable_agx_arm', 'enable_agx_arm'),
            ('move_home', 'move_home')
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        log_level_arg,
        can_port_arg,
        auto_enable_arg,
        arm_type_arg,
        end_effector_type_arg,
        use_upper_ik_arg,
        urdf_path_arg,
        agx_arm_node
    ])
