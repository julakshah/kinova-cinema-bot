""" module to contain all kinematics code for gen3_lite robot"""

import numpy as np
import time
import math

class Gen3LiteKinematics:
    """define the dh and position solver"""

    def __init__(self):
        # define the joint angles (thetas) in radians
        self.theta = [0, 0, 0, 0, 0, 0]

        # DH parameters for gen3_lite robot arm
        # Order: alpha, a, d, theta
        self.dh = [ 
            [np.pi / 2, 0, (0.1283 + 0.115), self.theta[0]],
            [np.pi, 0.280, 0.030, self.theta[1] + (np.pi / 2)],
            [np.pi / 2, 0, 0.020, self.theta[2] + (np.pi / 2)],
            [np.pi / 2, 0, (0.140 + 0.105), self.theta[3] + (np.pi / 2)],
            [np.pi / 2, 0, (0.0285 + 0.0285), self.theta[4] + np.pi],
            [0, 0, (0.105 + 0.130), self.theta[5] + (np.pi / 2)]
        ]
        
        # Radians
        self.joint_limits = [
            (-2.689, 2.689),
            (-2.621, 2.621),
            (-2.621, 2.621),
            (-2.600, 2.600),
            (-2.531, 2.530),
            (-2.600, 2.600)
        ]

        self.ndof = 6
        self.ee = EndEffector()
        self.points = [None] * (self.ndof + 1)
        self.T = np.zeros((self.ndof, 4, 4))
        self.calc_robot_points()
        self.last_time = time.time()
        
        

    def jacobian(self):
        """
        Define the jacobian for current dh.
        
        Creates the Jacobian for the gen3_lite dh table. Each column represents
        the cartesian movement of a point in relation to it's center about it's
        axis of rotation.
        """

        dh = [ 
            [np.pi / 2, 0, (0.1283 + 0.115), self.theta[0]],
            [np.pi, 0.280, 0.030, self.theta[1] + (np.pi / 2)],
            [np.pi / 2, 0, 0.020, self.theta[2] + (np.pi / 2)],
            [np.pi / 2, 0, (0.140 + 0.105), self.theta[3] + (np.pi / 2)],
            [np.pi / 2, 0, (0.0285 + 0.0285), self.theta[4] + np.pi],
            [0, 0, (0.105 + 0.130), self.theta[5] + (np.pi / 2)]
        ]
        # compute the cumulative transformation
        T_cumulative = [np.eye(4)]
        for i, row in enumerate(dh):
            Ti = dh_to_h(row)
            T_cumulative.append(T_cumulative[-1] @ Ti)
        
        # compute difference in joint and EE position (r)

        J = np.zeros((3, self.ndof))
        for i in range(self.ndof):
            r = (T_cumulative[i] - T_cumulative[-1]) @ np.array([0, 0, 0, 1])
            z = T_cumulative[i][:3, :3] @ np.array([0, 0, 1])

            J[:, i] = np.cross(z, r[:3])

        return J # the linear velo component of Jacobian
    
    def vinv_jacobian(self, J, pseudo=False):
        """define the inverse jacobian for current dh"""

        if pseudo:
            return np.linalg.pinv(J)
        return np.linalg.inv(J)

    def damped_inv_jacobian(self, damping_factor=1):
        """define the damped inverse jacobian for current dh"""

        J = self.jacobian()

        JT = np.transpose(J)
        I = np.eye(3)
        return JT @ np.linalg.inv(J @ JT + (damping_factor**2)*I)

    def position_fk(self):
        """find EE position given joint angles"""
         # Compute the transformation matrices
        for i in range(self.ndof):
            self.T[i] = dh_to_h(self.dh[i])

        # Calculate robot points (positions of joints)
        self.calc_robot_points()

     

    def velocity_fk(self, desired_vel):
        """
        move in cartesion coordinates by desired velocity
        
        Args:
            desired_vel: a list of floats giving the magnitude of desired
                    veloctiy (m/s)
        """
        time

        if all(th == 0.0 for th in self.theta):
            self.theta = [0.0 + np.random.rand() * .001 for _ in range(self.theta)]

        vel = np.array(desired_vel)
        # print(f"desired vel: {vel}")
        J = self.damped_inv_jacobian()
        # print(f"J is {J}")
        theta_dot = J @ vel
        # print(f"theta dot is {theta_dot}")

        # consider making this list comprehension
        for i, ang_vel in enumerate(theta_dot):
            # make sure it doesn't try
            # over max velocity of 1 rad/s
            if ang_vel > 1:
                print("reducing velocity to one rad/s")
                theta_dot[i] = 1

        for i in range(self.ndof):
            self.theta[i] += .05 * theta_dot[i]
        
        return self.theta
        # potential different implementation with theta_dot return
        # return theta_dot


    def check_dh_conversion(self):
        for i, row in enumerate(self.dh):
            print(f"row is {row}")
            Ti = dh_to_h(row)
            print(f"transform {i} is: {Ti}")


    def analytical_ik(self): 
        """analytically solve for the joint angles of a desired position"""

    def numerical_ik(self):
        """
        Numerically solve for the joint positions of a desired EE pose
        using the Newton-Raphson method of gradient descent.
        """
    def calc_robot_points(self):
        """Calculates the main arm points using the current joint angles"""

        # Initialize points[0] to the base (origin)
        self.points[0] = np.array([0, 0, 0, 1])

        # Precompute cumulative transformations to avoid redundant calculations
        T_cumulative = [np.eye(4)]
        for i in range(self.ndof):
            T_cumulative.append(T_cumulative[-1] @ self.T[i])

        # Calculate the robot points by applying the cumulative transformations
        for i in range(1, self.ndof+1): # not sure fi this is correct
            self.points[i] = T_cumulative[i] @ self.points[0]

        # Calculate EE position and rotation
        self.EE_axes = T_cumulative[-1] @ np.array(
            [0.075, 0.075, 0.075, 1]
        )  # End-effector axes
        self.T_ee = T_cumulative[-1]  # Final transformation matrix for EE

        # Set the end effector (EE) position
        self.ee.x, self.ee.y, self.ee.z = self.points[-1][:3]

        # Extract and assign the RPY (roll, pitch, yaw) from the rotation matrix
        rpy = rotm_to_euler(self.T_ee[:3, :3])
        self.ee.rotx, self.ee.roty, self.ee.rotz = rpy[0], rpy[1], rpy[2]

        # Calculate the EE axes in space (in the base frame)
        self.EE = [self.ee.x, self.ee.y, self.ee.z]
        # print(f"{self.points=}")
        print("self.ee", self.EE)
        self.EE_axes = np.array(
            [self.T_ee[:3, i] * 0.075 + self.points[-1][:3] for i in range(3)]
        )
     

    def calc_ik_kinematics(self, des_pos: list, tol=0.01, ilimit=50):
        """
        Numerically solve for joint angles for a desired EE position.
        """
        des_pos = np.array(des_pos)
        max_step = 0.1

        for i in range(ilimit):
            self.update_dh()
            self.position_fk()

            pos_current = np.array([self.ee.x, self.ee.y, self.ee.z])
            error = des_pos - pos_current

            print(f"[{i}] self.ee", pos_current)

            if np.linalg.norm(error) < tol:
                print("Converged!")
                break

            J_inv = self.damped_inv_jacobian()  # More stable than pseudoinverse
            delta_theta = J_inv @ error

            # Clamp the step to avoid overshooting
            delta_theta = np.clip(delta_theta, -max_step, max_step)

           # Update joint angles
            self.theta = [self.theta[j] + delta_theta[j] for j in range(self.ndof)]

            # Enforce hardware joint limits
            for j in range(self.ndof):
                lower, upper = self.joint_limits[j]
                self.theta[j] = np.clip(self.theta[j], lower, upper)


        return self.theta

    def update_dh(self):
        """Updates DH table with current theta values"""
        self.dh = [ 
            [np.pi / 2, 0, (128.3 + 115), self.theta[0]],
            [np.pi, 280, 30, self.theta[1] + np.pi / 2],
            [np.pi / 2, 0, 20, self.theta[2] + np.pi / 2],
            [np.pi / 2, 0, (140 + 105), self.theta[3] + np.pi / 2],
            [np.pi / 2, 0, (28.5 + 28.5), self.theta[4] + np.pi],
            [0, 0, (105 + 130), self.theta[5] + np.pi / 2]
        ]
        
        

        
        

def rotm_to_euler(R) -> tuple:
    """Converts a rotation matrix to Euler angles (roll, pitch, yaw).

    Args:
        R (np.ndarray): A 3x3 rotation matrix.

    Returns:
        tuple: Roll, pitch, and yaw angles (in radians).
    
    """
    r11 = R[0,0] if abs(R[0,0]) > 1e-7 else 0.0
    r12 = R[0,1] if abs(R[0,1]) > 1e-7 else 0.0
    r21 = R[1,0] if abs(R[1,0]) > 1e-7 else 0.0
    r22 = R[1,1] if abs(R[1,1]) > 1e-7 else 0.0
    r32 = R[2,1] if abs(R[2,1]) > 1e-7 else 0.0
    r33 = R[2,2] if abs(R[2,2]) > 1e-7 else 0.0
    r31 = R[2,0] if abs(R[2,0]) > 1e-7 else 0.0

    if abs(r31) != 1:
        roll = math.atan2(r32, r33)        
        yaw = math.atan2(r21, r11)
        denom = math.sqrt(r11 ** 2 + r21 ** 2)
        pitch = math.atan2(-r31, denom)
    
    elif r31 == 1:
        # pitch is close to -90 deg, i.e. cos(pitch) = 0.0
        # there are an infinitely many solutions, so we choose one possible solution where yaw = 0
        pitch, yaw = -np.pi/2, 0.0
        roll = -math.atan2(r12, r22)
    
    elif r31 == -1:
        # pitch is close to 90 deg, i.e. cos(pitch) = 0.0
        # there are an infinitely many solutions, so we choose one possible solution where yaw = 0
        pitch, yaw =np.pi/2, 0.0
        roll = math.atan2(r12, r22)
        

    return roll, pitch, yaw

def dh_to_h(dh_i: list):
    alpha, a, d, theta = dh_i[0], dh_i[1], dh_i[2], dh_i[3]
    T = np.array([
        [np.cos(theta), -1 * np.cos(alpha) * np.sin(theta), np.sin(alpha) * np.sin(theta), a * np.cos(theta)],
        [np.sin(theta), np.cos(alpha) * np.cos(theta), -1 * np.sin(alpha) * np.cos(theta), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return T


class EndEffector:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rotx: float = 0.0
    roty: float = 0.0
    rotz: float = 0.0
    
def calc_ik_kinematics(self, EE: EndEffector, tol=.01, ilimit = 50):
    
    des_pos = [EE.x, EE.y, EE.z]
    # cur_pos =  # get end effector position
    # error = des_pos - cur_pos
    
    # make sure that the loop doesn't run forever by defining iteration limit
    # for _ in range(ilimit):
    #         # solve for error
    #         pos_current = self.solve_forward_kinematics(self.theta)
    #         error = pos_des - pos_current[0:3]
    #         # If error outside tol, recalculate theta (Newton-Raphson)
    #         if np.linalg.norm(error) > tol:
    #             self.theta = self.theta + np.dot(
    #                 self.inverse_jacobian(pseudo=True), error
    #             )
    #         # If error is within tolerence: break
    #         else:
    #             break