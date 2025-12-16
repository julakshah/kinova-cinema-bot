#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs import Pose
import queue
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
FaceLandmarkerResult = mp.tasks.vision.FaceLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

model_path = ""


class Vision(Node):

    def __init__(self):
        super().__init__("vision")
        self.pub_target_ = self.create_publisher(Pose, "target_pose", 10)

    def loop(self):
        while True:
            if self.cv_result != FaceLandmarkerOptions():
                self.cv_result

    def vision_setup(self):
        def vision_callback(
            result: mp.tasks.vision.HandLandmarkerResult,  # type: ignore
            output_image: mp.Image,  # pylint: disable=unused-argument
            timestamp_ms: int,  # pylint: disable=unused-argument
        ):
            self.cv_result = result

        options = FaceLandmarkerOptions(
            running_mode=VisionRunningMode.LIVE_STREAM,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            output_face_blendshapes=False,
            output_facial_transformation_matrixes=False,
            result_callback=vision_callback,
        )


if __init__ == "__main__":
    rclpy.init()
    vision_node = Vision()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown(vision_node)
