from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node


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

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Absolute path to robot URDF file",
    )
    
    publish_calibrated_arg = DeclareLaunchArgument(
        "publish_calibrated",
        default_value="false",
        description="Publish /calibrated std_msgs/msg/Bool once (for legacy workflows)",
    )

    model = LaunchConfiguration("model")
    entity = LaunchConfiguration("arm_type")
    publish_calibrated = LaunchConfiguration("publish_calibrated")

    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        )
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=["-entity", entity, "-file", model],
        output="screen",
    )

    publish_calibrated_once = ExecuteProcess(
        condition=IfCondition(publish_calibrated),
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/calibrated",
            "std_msgs/msg/Bool",
            "{data: true}",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            arm_type,
            model_arg,
            publish_calibrated_arg,
            gazebo_launch,
            spawn_entity,
            publish_calibrated_once,
        ]
    )


