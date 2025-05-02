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
        #print("EVENT : " + \
        #      Base_pb2.ActionEvent.Name(notification.action_event))
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

    # target_angles = [0,0,0,0,0,0]
    target_angles = [0,345,75,0,300,0]

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
    measured_angles = base.GetMeasuredJointAngles()
    theta_list = base_to_robot_theta(measured_angles)
    
    print("Robot joint angles after moving to home:")
    for idx, theta in enumerate(theta_list):
        print(f"Joint {idx + 1}: {np.rad2deg(theta):.2f} degrees")
    base.Unsubscribe(notification_handle)

def base_to_robot_theta(base_dict):
    """
    Converts the dictionary of joint angles given by
    kinova base into a list for the robot kinematics to work.
    
    Args:
        base_dict: dictionary containing the joint angles of robot
            return from base.
            
    Returns:
        a list of joint angles that the robot is at.
    """
    # defines empty list for current theta values
    cur_theta = []
    # goes through the weird data structure and grabs each joint angle iteratively and appends to list
    for joint_angle in base_dict.joint_angles:
            cur_theta.append(np.deg2rad(joint_angle.value))
            # print(f"Joint {joint_angle.joint_identifier}: {joint_angle.value:.2f} degrees")

    return cur_theta


def move_to_angle(base, angles):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    # action_list = base.ReadAllActions(action_type)
    action_handle = None

    # Create the action message
    action = Base_pb2.Action()
    action.name = "Custom joint angles"
    action.application_data = ""

    target_angles = angles
    target_angles_deg = [np.rad2deg(a) for a in target_angles]
    print("Sending target angles (deg):", [f"{a:.2f}" for a in target_angles_deg])
    for i, angle in enumerate(target_angles_deg):
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
        
    # Instantiate the classes for our kinematics and controller
    controller = TeleopController()
    robot = kinematics.Gen3LiteKinematics()
    # robot.check_dh_conversion()
    
    print(robot.ee)
    # Connect to robot
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        move_to_home_position(base)

        # calculate where the robot actually is 
        robot.position_fk()
        iteration = 0

        while controller.running:
            
            # get current joint angles
            print(iteration)
            robot.check_dh_conversion()
            iteration = iteration + 1
            joint_angles = base.GetMeasuredJointAngles()
            robot.theta = base_to_robot_theta(joint_angles)

            if controller.target_ik is not None:
                print("Running numerical IK kinematics...")
                angles = robot.calc_ik_kinematics(controller.target_ik, tol=0.01, ilimit=50)
                move_to_angle(base, angles)
                # Reset the target after processing
                controller.target_ik = None
                robot.position_fk()


            # Example: apply velocity command
            velocity = controller.get_velocity_command()
            if velocity != [0, 0, 0]:
                # 1/40 is from 40hz refresh of high level control
                angles = robot.velocity_fk(velocity)
                move_to_angle(base, angles)
                
                
            # print(f"robot joints are: {robot.theta}")

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
        self.running = True
        self.target_ik = None  # New: store IK target when 'v' is pressed

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
                self.v[2] = -1
            elif key.char == 's':
                self.v[2] = 1
            elif key.char == 'q':  # exit on 'q'
                self.running = False
            elif key.char == 'v':
                self.target_ik = [450, 200, 450]
                print("IK target set to [450, 200, 450]") # similar to home position of bot normally
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
        # print(f"Velocity: {self.v}")  # replace with command to robot
        return self.v


    

if __name__ == "__main__":
    main()

