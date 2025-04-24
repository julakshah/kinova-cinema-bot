import sys
import os
import time
import threading
import math
import numpy as np
from pynput import keyboard
import kinematics
import utilities
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
    
    # Parse arguments
    args = utilities.parseConnectionArguments()
        
    # Create connection to the device and get the router      
            
    controller = TeleopController()
    robot = kinematics.Gen3LiteKinematics()
    
    print(robot.ee)
    # Connect to robot
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        
        
        print(joint_angles.joint_angles)
        
        # defines empty list for current theta values
        cur_theta = []
        # goes through the weird data structure and grabs each joint angle iteratively and appends to list
        for joint_angle in joint_angles.joint_angles:
            cur_theta.append(joint_angle.value)
            print(f"Joint {joint_angle.joint_identifier}: {joint_angle.value:.2f} degrees")
        
        # robot angles = current theta values just assigned
        robot.t = cur_theta

        # calculate where the robot actually is 
        robot.position_fk()
        
        base = BaseClient(router)
        move_to_home_position(base)
        joint_angles = base.GetMeasuredJointAngles()

       
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

