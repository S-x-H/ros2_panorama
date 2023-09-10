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
        description="Full filepath to .json containing all camera calibration values. Can be left blank to skip undistortion.",
    )
    launch_items.append(camera_calibrations_path_arg)

    left_camera_name_arg = DeclareLaunchArgument(
        "left_camera_name",
        default_value="camera_3",
        description="When doing undistortion: name of left camera in camera calibration json.",
    )
    launch_items.append(left_camera_name_arg)

    centre_camera_name_arg = DeclareLaunchArgument(
        "centre_camera_name",
        default_value="camera_2",
        description="When doing undistortion: name of centre camera in camera calibration json.",
    )
    launch_items.append(centre_camera_name_arg)

    right_camera_name_arg = DeclareLaunchArgument(
        "right_camera_name",
        default_value="camera_1",
        description="When doing undistortion: name of right camera in camera calibration json.",
    )
    launch_items.append(right_camera_name_arg)

    attempt_stitching_arg = DeclareLaunchArgument(
        "attempt_stitching",
        default_value="False",
        description="True to attempt image stitching instead of mere stacking, will still stack if stitching fails.",
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

    stitching_timeout_arg = DeclareLaunchArgument(
        "stitching_timeout",
        default_value="0.5",
        description="To continue running in real-time, length(s) to attempt stitching per frame before giving up, if stitching is slow.",
    )
    launch_items.append(stitching_timeout_arg)

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
            {"left_camera_name": LaunchConfiguration("left_camera_name")},
            {"centre_camera_name": LaunchConfiguration("centre_camera_name")},
            {"right_camera_name": LaunchConfiguration("right_camera_name")},
            {"attempt_stitching": LaunchConfiguration("attempt_stitching")},
            {"stitching_detector": LaunchConfiguration("stitching_detector")},
            {"stitching_threshold": LaunchConfiguration("stitching_threshold")},
            {"stitching_timeout": LaunchConfiguration("stitching_timeout")},
        ],
        remappings=[
            ("panorama", "cams123/panorama"),
            ("camera_left", "/platypus/camera_3/dec/manual_white_balance"),
            ("camera_centre", "/platypus/camera_2/dec/manual_white_balance"),
            ("camera_right", "/platypus/camera_1/dec/manual_white_balance"),
        ],
    )
    launch_items.append(image_merger)

    return LaunchDescription(launch_items)
