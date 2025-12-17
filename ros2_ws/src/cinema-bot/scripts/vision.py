#!/usr/bin/env python3

import queue
import threading
import time
import os
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
FaceLandmarkerResult = mp.tasks.vision.FaceLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

VID_CHANNEL = 0

class Vision(Node):

    def __init__(self):
        super().__init__("vision")
        self.pub_target_ = self.create_publisher(Pose, "target_pose", 10)

        self.detector, self.cap = self.vision_setup()
        self.cv_result=None

        self.timer = self.create_timer(.01, self.loop)
        

    def loop(self):
        success, image = self.cap.read()
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        self.detector.detect_async(image=mp_image, timestamp_ms=int(time.time() * 1000))
        msg = Pose()
        if self.cv_result is not None and self.cv_result.face_landmarks != []:
            print(f"{self.cv_result.face_landmarks[0][4]}\n\n")
            result = self.cv_result.face_landmarks[0][4]
            x, y, z = result.x, result.y, result.z
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
        else:
            msg.position.x = .5; msg.position.y = .5; msg.position.z = .5
        self.pub_target_.publish(msg)


    def vision_setup(self):
        cap = self.create_cap(0)
        share_dir = get_package_share_directory("cinema-bot")
        model_path = os.path.join(share_dir,"config","face_landmarker.task")
        print(f"model path is: {model_path}")

        def vision_callback(
            result: mp.tasks.vision.FaceLandmarkerResult,  # type: ignore
            output_image: mp.Image,  # pylint: disable=unused-argument
            timestamp_ms: int,  # pylint: disable=unused-argument
        ):
            self.cv_result = result

        options = FaceLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            output_face_blendshapes=False,
            output_facial_transformation_matrixes=False,
            result_callback=vision_callback,
        )

        detector = vision.FaceLandmarker.create_from_options(options)

        return detector, cap

    def create_cap(self, attempt):
        """
        creates the videocapture element and checks for failed video opening.

        Args:
            attempt: an integer representing the current attempt
        """
        cap = cv2.VideoCapture(VID_CHANNEL)  # pylint: disable=no-member
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


if __name__ == "__main__":
    rclpy.init()
    vision_node = Vision()
    mp_thread = threading.Thread(target=vision_node)
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown(vision_node)
