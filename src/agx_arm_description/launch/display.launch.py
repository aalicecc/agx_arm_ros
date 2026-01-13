from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        description="Absolute path to an RViz2 config file. If empty, RViz2 starts with defaults.",
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start joint_state_publisher_gui (true) or joint_state_publisher (false)",
    )

    model = LaunchConfiguration("model")
    rvizconfig = LaunchConfiguration("rvizconfig")
    use_gui = LaunchConfiguration("use_gui")

    # launch.substitutions.Command concatenates list items without inserting spaces,
    # so we include a trailing space after 'cat ' to avoid 'cat/path/to/file' errors.
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rvizconfig],
        output="screen",
    )

    return LaunchDescription(
        [
            arm_type,
            model_arg,
            rvizconfig_arg,
            use_gui_arg,
            jsp_gui_node,
            jsp_node,
            rsp_node,
            rviz,
        ]
    )


