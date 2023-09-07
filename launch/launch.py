from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from typing import List, Any


def generate_launch_description():
    launch_items: List[Any] = list()

    display_img_window_arg = DeclareLaunchArgument(
        "display_img_window",
        default_value="False",
        description="True to display panorama image being published in window.",
    )
    launch_items.append(display_img_window_arg)

    camera_calibrations_path_arg = DeclareLaunchArgument(
        "camera_calibrations_path",
        default_value="intrinsics_extrinsics.json",
        description="Full filepath to .json containing all camera calibration values.",
    )
    launch_items.append(camera_calibrations_path_arg)

    attempt_stitching_arg = DeclareLaunchArgument(
        "attempt_stitching",
        default_value="False",
        description="True to do image stitching instead of mere stacking.",
    )
    launch_items.append(attempt_stitching_arg)

    stitching_detector_arg = DeclareLaunchArgument(
        "stitching_detector",
        default_value="orb",
        description="When doing stitching: 'orb', 'sift', 'brisk', or 'akaze'.",
    )
    launch_items.append(stitching_detector_arg)

    stitching_threshold_arg = DeclareLaunchArgument(
        "stitching_threshold",
        default_value="0.2",
        description="When doing stitching: feature matching threshold between 0 - 1.",
    )
    launch_items.append(stitching_threshold_arg)

    image_merger = Node(
        package="ros2_panorama",
        executable="image_merger",
        parameters=[
            {"display_img_window": LaunchConfiguration("display_img_window")},
            {
                "camera_calibrations_path": LaunchConfiguration(
                    "camera_calibrations_path"
                )
            },
            {"attempt_stitching": LaunchConfiguration("attempt_stitching")},
            {"stitching_detector": LaunchConfiguration("stitching_detector")},
            {"stitching_threshold": LaunchConfiguration("stitching_threshold")},
        ],
        remappings=[
            ("panorama", "cams123/panorama"),
            ("cam1", "/platypus/camera_1/dec/manual_white_balance"),
            ("cam2", "/platypus/camera_2/dec/manual_white_balance"),
            ("cam3", "/platypus/camera_3/dec/manual_white_balance"),
        ],
    )
    launch_items.append(image_merger)

    return LaunchDescription(launch_items)
