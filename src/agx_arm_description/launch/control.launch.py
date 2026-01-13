from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_type = DeclareLaunchArgument(
        "arm_type",
        default_value="nero",
        description="Arm model to display. Options: nero, piper"
    )

    pkg_share = get_package_share_directory("agx_arm_description")
    
    default_model = PathJoinSubstitution([
        pkg_share,
        LaunchConfiguration("arm_type"),
        "urdf",
        "nero.urdf"
    ])
    default_rvizconfig = PathJoinSubstitution([
        pkg_share,
        "rviz", "display.rviz"
    ])

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Absolute path to robot URDF file",
    )
    rvizconfig_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=default_rvizconfig,
        description="Absolute path to an RViz2 config file",
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Use joint_state_publisher_gui (true) or joint_state_publisher (false)",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2",
    )
    use_relay_arg = DeclareLaunchArgument(
        "use_relay",
        default_value="true",
        description="Relay /joint_states -> /control/joint_states",
    )
    source_topic_arg = DeclareLaunchArgument(
        "source_joint_states",
        default_value="/joint_states",
        description="Source joint states topic",
    )
    control_topic_arg = DeclareLaunchArgument(
        "control_joint_states",
        default_value="/control/joint_states",
        description="Control joint states topic (downstream controller subscribes to this)",
    )

    model = LaunchConfiguration("model")
    rvizconfig = LaunchConfiguration("rvizconfig")
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_relay = LaunchConfiguration("use_relay")
    source_topic = LaunchConfiguration("source_joint_states")
    control_topic = LaunchConfiguration("control_joint_states")

    # NOTE: Command concatenates list items without inserting spaces.
    robot_description = ParameterValue(Command(["cat ", model]), value_type=str)

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"publish_robot_description": True},
        ],
        output="screen",
    )

    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
        output="screen",
    )
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(use_gui),
        output="screen",
    )

    # Relay to controller topic while keeping /joint_states for robot_state_publisher/RViz.
    relay = ExecuteProcess(
        condition=IfCondition(use_relay),
        cmd=["ros2", "run", "topic_tools", "relay", source_topic, control_topic],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rvizconfig],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            arm_type,
            model_arg,
            rvizconfig_arg,
            use_gui_arg,
            use_rviz_arg,
            use_relay_arg,
            source_topic_arg,
            control_topic_arg,
            jsp_gui_node,
            jsp_node,
            rsp_node,
            relay,
            rviz,
        ]
    )


