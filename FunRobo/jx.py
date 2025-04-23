#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading
import math
import numpy as np
from pynput import keyboard

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2, Common_pb2

# Waiting time between actions (in milliseconds)
ACTION_WAITING_TIME = 1

# Create closure to set finished to true after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    finished -- list of bool to affect the result (of size 1)
                (will be set to True when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    # action_list = base.ReadAllActions(action_type)
    action_handle = None

    # Create the action message
    action = Base_pb2.Action()
    action.name = "Custom joint angles"
    action.application_data = ""


    # Specify that this is a REACH_JOINT_ANGLES action
    # action.action_type = Base_pb2.REACH_JOINT_ANGLES



    target_angles = [10.0, 90.0, 30.0, 40.0, 50.0, 60.0]

    for i, angle in enumerate(target_angles):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = i + 1
        joint_angle.value = angle  # correct field name

    # if action_handle == None:
    #     print("Can't reach safe position. Exiting")
    #     sys.exit(1)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteAction(action)

    e.wait()

    base.Unsubscribe(notification_handle)

def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    ul1, ul2, ul3, ul4, ul5, ul6, ul7 = 128.3, 115, 280, 140, 108, 108, 130
    sl1, sl2, sl3, sl4 = 30, 20, 28.5, 28.5
    t1, t2, t3, t4, t5, t6 = 0,0,0,0,0,0 
    
    dhtable = [
        [t1, 128.3 + 115.0, 0.0, math.pi / 2],
        [t2 + math.pi / 2, 30.0, 280.0, math.pi],
        [t3 + math.pi / 2, 20.0, 0.0, math.pi / 2],
        [t4 + math.pi / 2, 140.0 + 105.0, 0.0, math.pi / 2],
        [t5 + math.pi, 28.5 + 28.5, 0.0, math.pi / 2],
        [t6 + math.pi / 2, 105.0 + 130.0, 0.0, 0.0]
    ]

    htmMatrices = []
    
    # for i in range(len(dhtable)):
    #     htmMatrices.append(dh_to_matrix(dhtable[i]))
        
    # hm = htmMatrices[0] * htmMatrices[1] *  htmMatrices[2]  *  htmMatrices[3]  *  htmMatrices[4] *  htmMatrices[5] 
        
    # Create connection to the device and get the router      
            
    controller = TeleopController()

    # Connect to robot
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        move_to_home_position(base)
        joint_angles = base.GetMeasuredJointAngles()

        for joint_angle in joint_angles.joint_angles:
            print(f"Joint {joint_angle.joint_identifier}: {joint_angle.value:.2f} degrees")
        while controller.running:
            # Example: apply velocity command
            velocity = controller.get_velocity_command()

            # TODO: Create and send velocity command here using `velocity` and `base`
            time.sleep(0.1)  # small delay to avoid flooding

        print("Teleop ended.")



# things we need to make actually write our own code

"""
getAllJointAngles
grab DH table
connect controller/keyboard with our commands
"""

def dh_to_matrix(dh_params: list) -> np.ndarray:
    """Converts Denavit-Hartenberg parameters to a transformation matrix.

    Args:
        dh_params (list): Denavit-Hartenberg parameters [theta, d, a, alpha].

    Returns:
        np.ndarray: A 4x4 transformation matrix.
    """
    theta, d, a, alpha = dh_params
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                 d],
        [0,              0,                             0,                             1]
    ])
class TeleopController:
    def __init__(self):
        self.v = [0, 0, 0]  # x, y, z velocity commands
        self.running = True
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.v[1] = 1
            elif key == keyboard.Key.down:
                self.v[1] = -1
            elif key == keyboard.Key.left:
                self.v[0] = -1
            elif key == keyboard.Key.right:
                self.v[0] = 1
            elif key.char == 'w':
                self.v[2] = 1
            elif key.char == 's':
                self.v[2] = -1
            elif key.char == 'q':  # exit on 'q'
                self.running = False
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key == keyboard.Key.up or key == keyboard.Key.down:
                self.v[1] = 0
            elif key == keyboard.Key.left or key == keyboard.Key.right:
                self.v[0] = 0
            elif key.char == 'w' or key.char == 's':
                self.v[2] = 0
        except AttributeError:
            pass

    def get_velocity_command(self):
        # convert self.v to your robot's expected format (e.g., Base_pb2.TwistCommand)
        print(f"Velocity: {self.v}")  # replace with command to robot
        return self.v


if __name__ == "__main__":
    main()
