#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from three_sensor_overlay.calibration_io import (
    load_sensor_extrinsics,
    render_standalone_urdf,
)


def calibrated_robot_description(context):
    """Build robot_description directly from the same files used by the overlay."""
    lidar_path = LaunchConfiguration("lidar_calibration_path").perform(context)
    camera_path = LaunchConfiguration("camera_calibration_path").perform(context)
    extrinsics = load_sensor_extrinsics(lidar_path, camera_path)
    get_logger("three_sensor_overlay").info(
        f"Robot TF LiDAR calibration: {extrinsics.lidar_calibration_path}"
    )
    get_logger("three_sensor_overlay").info(
        f"Robot TF camera calibration: {extrinsics.camera_calibration_path} "
        f"(results.{extrinsics.camera_transform_key})"
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="three_sensor_robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": render_standalone_urdf(extrinsics),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            condition=IfCondition(LaunchConfiguration("publish_sensor_tf")),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("three_sensor_overlay"))
    rviz_config = str(share_dir / "rviz" / "three_sensor_overlay.rviz")

    bag_path = LaunchConfiguration("bag_path")
    play_bag = LaunchConfiguration("play_bag")
    play_rate = LaunchConfiguration("play_rate")
    start_offset = LaunchConfiguration("start_offset")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    use_apriltag = LaunchConfiguration("apriltag")
    assume_depth_aligned = LaunchConfiguration("assume_depth_aligned")
    tag_family = LaunchConfiguration("tag_family")
    tag_size = LaunchConfiguration("tag_size_m")
    lidar_calibration_path = LaunchConfiguration("lidar_calibration_path")
    camera_calibration_path = LaunchConfiguration("camera_calibration_path")

    declarations = [
        DeclareLaunchArgument(
            "bag_path",
            default_value="",
            description="ROS 2 bag directory. Required only when play_bag:=true.",
        ),
        DeclareLaunchArgument("play_bag", default_value="false"),
        DeclareLaunchArgument("play_rate", default_value="1.0"),
        DeclareLaunchArgument("start_offset", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("apriltag", default_value="true"),
        DeclareLaunchArgument(
            "lidar_calibration_path",
            default_value="",
            description="Absolute path to extrinsic_lidar91_to_lidar90.yaml.",
        ),
        DeclareLaunchArgument(
            "camera_calibration_path",
            default_value="",
            description="Absolute path to the accepted camera-LiDAR calib.json.",
        ),
        DeclareLaunchArgument(
            "publish_sensor_tf",
            default_value="true",
            description="Publish TF generated from the supplied calibration files.",
        ),
        DeclareLaunchArgument(
            "assume_depth_aligned",
            default_value="false",
            description=(
                "Preview only: publish identity color-optical to depth-optical TF. "
                "The historical bag does not contain the factory transform."
            ),
        ),
        DeclareLaunchArgument(
            "tag_family",
            default_value="Standard41h12",
            description="Confirmed family in this bag: Standard41h12.",
        ),
        DeclareLaunchArgument(
            "tag_size_m",
            default_value="0.0872",
            description="Standard41h12 pose-corner square edge length in metres.",
        ),
    ]

    robot_state_publisher = OpaqueFunction(function=calibrated_robot_description)

    depth_identity_preview = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="uncalibrated_depth_identity_preview",
        output="screen",
        condition=IfCondition(assume_depth_aligned),
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "camera_color_optical_frame",
            "--child-frame-id",
            "camera_depth_optical_frame",
        ],
    )

    overlay_node = Node(
        package="three_sensor_overlay",
        executable="sensor_overlay_node",
        name="sensor_overlay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "lidar_calibration_path": lidar_calibration_path,
                "camera_calibration_path": camera_calibration_path,
            }
        ],
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        namespace="overlay",
        name="apriltag",
        output="screen",
        condition=IfCondition(use_apriltag),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_transport": "raw",
                "qos_profile": "sensor_data",
                "family": tag_family,
                "size": ParameterValue(tag_size, value_type=float),
                "max_hamming": 0,
                "detector.threads": 2,
                "detector.decimate": 1.5,
                "detector.blur": 0.0,
                "detector.refine": True,
                "detector.sharpening": 0.25,
                "pose_estimation_method": "pnp",
            }
        ],
        remappings=[
            ("image_rect", "/overlay/camera/image_rect"),
            ("camera_info", "/overlay/camera/camera_info"),
            ("detections", "/overlay/apriltag/detections"),
        ],
    )

    rviz = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="three_sensor_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_rviz),
            )
        ],
    )

    # Replaying only the six useful topics avoids pushing ~1.75 million IMU
    # messages through the graph. A delay gives subscribers and RViz time to start.
    bag_player = GroupAction(
        condition=IfCondition(play_bag),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--clock",
                    "100",
                    "--rate",
                    play_rate,
                    "--start-offset",
                    start_offset,
                    "--delay",
                    "3.0",
                    "--topics",
                    "/camera/color/camera_info",
                    "/camera/color/image_raw/compressed",
                    "/camera/depth/camera_info",
                    "/camera/depth/image_raw/compressedDepth",
                    "/lidar/lidar_90",
                    "/lidar/lidar_91",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        declarations
        + [
            robot_state_publisher,
            depth_identity_preview,
            overlay_node,
            apriltag_node,
            rviz,
            bag_player,
        ]
    )
