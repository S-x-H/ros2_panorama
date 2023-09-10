import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import json
import message_filters
import multiprocessing
import numpy as np
from sensor_msgs.msg import Image
from stitching import AffineStitcher

from cv2.typing import MatLike
from numpy.typing import NDArray
from typing import Dict, Any, Tuple, List


class ImageMerger(Node):
    def __init__(self):
        super().__init__("image_merger")

        # create merged panorama image publisher
        self.panorama_pub = self.create_publisher(Image, "panorama", 10)

        # create 3 camera image subscribers with single time synchronised callback
        self.cam_left_sub = message_filters.Subscriber(self, Image, "camera_left")
        self.cam_centre_sub = message_filters.Subscriber(self, Image, "camera_centre")
        self.cam_right_sub = message_filters.Subscriber(self, Image, "camera_right")

        self.declare_parameter("camera_sync_tolerance", 0.01)
        self.filter = message_filters.ApproximateTimeSynchronizer(
            [self.cam_left_sub, self.cam_centre_sub, self.cam_right_sub],
            queue_size=10,
            slop=self.get_parameter("camera_sync_tolerance")
            .get_parameter_value()
            .double_value,
        )
        self.filter.registerCallback(self.img_cb)

        # load camera calibrations
        self.declare_parameter("camera_calibrations_path", "")
        self.calibrations_available: bool = True
        try:
            camera_calibrations = json.load(
                open(
                    self.get_parameter("camera_calibrations_path")
                    .get_parameter_value()
                    .string_value
                )
            )

            self.declare_parameter("left_camera_name", "")
            self.declare_parameter("centre_camera_name", "")
            self.declare_parameter("right_camera_name", "")
            left_camera_name: str = (
                self.get_parameter("left_camera_name")
                .get_parameter_value()
                .string_value
            )
            centre_camera_name: str = (
                self.get_parameter("centre_camera_name")
                .get_parameter_value()
                .string_value
            )
            right_camera_name: str = (
                self.get_parameter("right_camera_name")
                .get_parameter_value()
                .string_value
            )

            # list all K and d matrices in left to right camera order
            self.K: List[NDArray[Any]] = list()
            self.d: List[NDArray[Any]] = list()
            for camera in [left_camera_name, centre_camera_name, right_camera_name]:
                K, d = self.read_camera_calibration(
                    camera_calibrations[camera]["intrinsics"]
                )
                self.K.append(K)
                self.d.append(d)

        except Exception:
            self.get_logger().error(
                "Camera names and calibrations file are missing or not valid. No image correction will be performed."
            )
            self.calibrations_available = False

        # create cv bridge
        self.br = CvBridge()

        # declare processing parameters that will be reloaded every callback to allow dynamic reconfiguration
        self.declare_parameter("attempt_stitching", True)
        self.declare_parameter("stitching_detector", "orb")
        self.declare_parameter("stitching_threshold", 0.2)
        self.declare_parameter("stitching_timeout", 0.5)
        self.declare_parameter("display_img_window", False)

        self.count = 0

    def read_camera_calibration(
        self, intrinsics: Dict[str, Any]
    ) -> Tuple[NDArray[Any], NDArray[Any]]:
        # construct K and d matrices from focal length, principal point, and distortion values in calibration file
        K: NDArray[Any] = np.array(
            [
                [intrinsics["focal_length"], 0, intrinsics["principal_point"]["x"]],
                [0, intrinsics["focal_length"], intrinsics["principal_point"]["y"]],
                [0, 0, 1],
            ]
        )
        d: NDArray[Any] = np.array(
            [
                intrinsics["distortion"]["radial"]["k1"],
                intrinsics["distortion"]["radial"]["k2"],
                intrinsics["distortion"]["tangential"]["p1"],
                intrinsics["distortion"]["tangential"]["p2"],
                intrinsics["distortion"]["radial"]["k3"],
            ]
        )
        return K, d

    def img_cb(
        self, cam_msg_left: Image, cam_msg_centre: Image, cam_msg_right: Image
    ) -> None:
        # list all images in left to right camera order, matching K and d matrices
        imgs: List[MatLike] = list()
        imgs.append(self.br.imgmsg_to_cv2(cam_msg_left, desired_encoding="passthrough"))
        imgs.append(
            self.br.imgmsg_to_cv2(cam_msg_centre, desired_encoding="passthrough")
        )
        imgs.append(
            self.br.imgmsg_to_cv2(cam_msg_right, desired_encoding="passthrough")
        )

        # undistort images, if available
        if self.calibrations_available:
            for i in range(3):
                h, w = imgs[i].shape[:2]
                newcamera, _ = cv2.getOptimalNewCameraMatrix(
                    self.K[i], self.d[i], (w, h), 0
                )
                imgs[i] = cv2.undistort(imgs[i], self.K[i], self.d[i], None, newcamera)

        # NOTE: stitching will perform poorly or fail completely if images have little overlapping area, or a low number of features
        # stacking will be performed anyway if stitching fails
        # it may be desirable to disable stitching if it succeeds but returns incorrect results, or if it repeatedly fails and slows down processing
        stitched = False

        # attempt stitching, if desired
        # reload attempt_stitching variable to allow for dynamic reconfiguration
        if self.get_parameter("attempt_stitching").get_parameter_value().bool_value:
            # reload processing variables to allow for dynamic reconfiguration
            self.settings: Dict[str, Any] = {
                "detector": self.get_parameter("stitching_detector")
                .get_parameter_value()
                .string_value,
                "confidence_threshold": self.get_parameter("stitching_threshold")
                .get_parameter_value()
                .double_value,
                "crop": False,
            }

            # attempt stitching for stitching_timeout seconds, or until return
            manager = multiprocessing.Manager()

            # pass in a list to hold return value, as more specific types are not supported
            return_list = manager.list()

            p = multiprocessing.Process(
                target=do_stitch, args=(self.settings, imgs, return_list)
            )
            p.start()

            # reload stitching_timeout variable to allow for dynamic reconfiguration
            p.join(
                self.get_parameter("stitching_timeout")
                .get_parameter_value()
                .double_value
            )
            if p.is_alive():
                # if stitching function has not returned within timeout limit, kill it
                p.terminate()

            # if successfully stitched image was returned
            if len(return_list) != 0:
                panorama: MatLike = return_list[0]
                stitched = True
            else:
                self.get_logger().error(
                    "Stitching failed or timed out. Stacking instead."
                )

        if not stitched:
            panorama: MatLike = cv2.hconcat(imgs)

        self.panorama_pub.publish(self.br.cv2_to_imgmsg(panorama))

        # reload display_img_window variable to allow for dynamic reconfiguration
        if self.get_parameter("display_img_window").get_parameter_value().bool_value:
            cv2.imshow("panorama", panorama)
            cv2.waitKey(1)


def do_stitch(
    settings: dict[str, Any], imgs: List[MatLike], return_list: List[MatLike]
):
    stitcher = AffineStitcher(**settings)
    try:
        stitched: MatLike = stitcher.stitch(imgs)
        return_list.append(stitched)
    except Exception:
        # if stitching fails, simply leave return_list empty
        return


def main():
    rclpy.init()
    stitcher = ImageMerger()
    rclpy.spin(stitcher)
    stitcher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
