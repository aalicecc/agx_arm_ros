from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"  

def generate_launch_description():

    # arg
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )

    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the AGX Arm node.'
    )

    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Publishing rate for the AGX Arm node.'
    )
    
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value= 'True',
        description='Automatically enable the AGX Arm node.'
    )

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='piper',
        description='Type of robotic arm.',
        choices=['piper', 'nero', 'piper_x', 'piper_h', 'piper_l']
    )

    speed_percent_arg = DeclareLaunchArgument(
        'speed_percent',
        default_value='100',
        description='Movement speed as a percentage of maximum speed.'
    )

    enable_timeout_arg = DeclareLaunchArgument(
        'enable_timeout',
        default_value='5.0',
        description='Timeout in seconds for arm enable/disable operations.'
    )

    # node
    agx_arm_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_ctrl_single_node',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),    
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'arm_type': LaunchConfiguration('arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
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

    return LaunchDescription([
        # arg
        log_level_arg,
        can_port_arg,
        pub_rate_arg,
        auto_enable_arg,
        arm_type_arg,
        speed_percent_arg,
        enable_timeout_arg,
        # node
        agx_arm_node
    ])
