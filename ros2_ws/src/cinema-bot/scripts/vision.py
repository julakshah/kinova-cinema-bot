#!/usr/bin/env python3

import queue
import threading
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs import Pose

BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
FaceLandmarkerResult = mp.tasks.vision.FaceLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

class Vision(Node):

    def __init__(self):
        super().__init__("vision")
        self.pub_target_ = self.create_publisher(Pose, "target_pose", 10)

        self.detector, self.cap = self.vision_setup()

        self.timer = self.create_timer(.01, self.loop)
        

    def loop(self):
        detector.detect_async()
        if self.cv_result != FaceLandmarkerOptions():
            self.cv_result

    def vision_setup(self):
        cap = self.create_cap(0)
        share_dir = get_package_share_directory("cinema-bot")
        model_path = os.path.join(share_dir,"config","blaze_face_short_range.tflite")

        def vision_callback(
            result: mp.tasks.vision.HandLandmarkerResult,  # type: ignore
            output_image: mp.Image,  # pylint: disable=unused-argument
            timestamp_ms: int,  # pylint: disable=unused-argument
        ):
            self.cv_result = result

        options = FaceLandmarkerOptions(
            running_mode=VisionRunningMode.LIVE_STREAM,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            result_callback=vision_callback,
        )

        detector = vision.FaceDetector.create_from_options(options)

        return detector, cap

    def create_cap(self, attempt):
        """
        creates the videocapture element and checks for failed video opening.

        Args:
            attempt: an integer representing the current attempt
        """
        cap = cv2.VideoCapture(self.channel)  # pylint: disable=no-member
        time.sleep(0.5)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            cap.set(cv2.CAP_PROP_FPS, 15)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return cap
        elif attempt == 10:
            print("video capture FAILED, closing")
        else:
            print("video capture open failed, please restart")
            self.create_cap(attempt=attempt + 1)


if __init__ == "__main__":
    rclpy.init()
    vision_node = Vision()
    mp_thread = threading.Thread(target=vision_node)
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown(vision_node)
