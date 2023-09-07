import rclpy
from rclpy.node import Node
from typing import Dict, Any, Tuple, List
import message_filters
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from numpy.typing import ArrayLike
from cv2.typing import MatLike
from stitching import AffineStitcher


class ImageMerger(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

        self.panorama_pub = self.create_publisher(Image, "panorama", 10)

        self.cam1_sub = message_filters.Subscriber(self, Image, "cam1")
        self.cam2_sub = message_filters.Subscriber(self, Image, "cam2")
        self.cam3_sub = message_filters.Subscriber(self, Image, "cam3")
        self.filter = message_filters.TimeSynchronizer(
            [self.cam1_sub, self.cam2_sub, self.cam3_sub], queue_size=10
        )
        self.filter.registerCallback(self.img_cb)

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
            self.K: List[ArrayLike] = list()
            self.d: List[ArrayLike] = list()
            for i in range(3):
                K, d = self.read_camera_calibration(
                    camera_calibrations[f"camera_{i + 1}"]["intrinsics"]
                )
                self.K.append(K)
                self.d.append(d)
        except Exception:
            self.get_logger().error(
                "Path to camera calibrations file is missing or not valid json. No correction will be performed."
            )
            self.calibrations_available = False

        self.br = CvBridge()

        self.declare_parameter("display_img_window", False)
        self.display_img_window: bool = (
            self.get_parameter("display_img_window").get_parameter_value().bool_value
        )

        self.declare_parameter("attempt_stitching", False)
        self.declare_parameter("stitching_detector", "orb")
        self.declare_parameter("stitching_threshold", 0.2)

    def read_camera_calibration(
        self, intrinsics: Dict[str, Any]
    ) -> Tuple[ArrayLike, ArrayLike]:
        K: ArrayLike = np.array(
            [
                [intrinsics["focal_length"], 0, intrinsics["principal_point"]["x"]],
                [0, intrinsics["focal_length"], intrinsics["principal_point"]["y"]],
                [0, 0, 1],
            ]
        )
        d: ArrayLike = np.array(
            [
                intrinsics["distortion"]["radial"]["k1"],
                intrinsics["distortion"]["radial"]["k2"],
                intrinsics["distortion"]["tangential"]["p1"],
                intrinsics["distortion"]["tangential"]["p2"],
                intrinsics["distortion"]["radial"]["k3"],
            ]
        )
        return K, d

    def img_cb(self, cam1: Image, cam2: Image, cam3: Image) -> None:
        imgs: List[MatLike] = list()
        imgs.append(self.br.imgmsg_to_cv2(cam1, desired_encoding="passthrough"))
        imgs.append(self.br.imgmsg_to_cv2(cam2, desired_encoding="passthrough"))
        imgs.append(self.br.imgmsg_to_cv2(cam3, desired_encoding="passthrough"))

        if self.calibrations_available:
            for i in range(3):
                h, w = imgs[i].shape[:2]
                newcamera, roi = cv2.getOptimalNewCameraMatrix(
                    self.K[i], self.d[i], (w, h), 0
                )
                imgs[i] = cv2.undistort(imgs[i], self.K[i], self.d[i], None, newcamera)

        if self.get_parameter("attempt_stitching").get_parameter_value().bool_value:
            settings = {
                "detector": self.get_parameter("stitching_detector")
                .get_parameter_value()
                .string_value,
                "confidence_threshold": self.get_parameter("stitching_threshold")
                .get_parameter_value()
                .double_value,
            }
            stitcher = AffineStitcher(**settings)
            pan = stitcher.stitch_verbose([cam3, cam2, cam1])
        else:
            pan = cv2.hconcat([imgs[2], imgs[1], imgs[0]])

        self.panorama_pub.publish(self.br.cv2_to_imgmsg(pan))

        if self.display_img_window:
            cv2.imwrite("panorama.jpg", pan)
            cv2.waitKey(1)


def main():
    rclpy.init()
    stitcher = ImageMerger()
    rclpy.spin(stitcher)
    stitcher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
