""" module to contain all kinematics code for gen3_lite robot"""

import numpy as np

class Gen3LiteKinematics:
    """define the dh and position solver"""

    def __init__(self):
        # define the joint angles (thetas) in radians
        self.t = [0, 0, 0, 0, 0, 0]

        # DH parameters for gen3_lite robot arm
        self.dh = [ 
            [np.pi() / 2, 0, (128.3 + 115), self.t[1]],
            [np.pi(), 280, 30, self.t[2] + np.pi() / 2],
            [np.pi() /2, 0, 20, self.t[3] + np.pi() / 2],
            [np.pi() / 2, 0, (140 + 105), self.t[4] + np.pi() / 2],
            [np.pi() / 2, 0, (28.5 + 28.5), self.t[5] + np.pi()],
            [0, 0, (105 + 130), self.t[6] + np.pi() / 2]
        ]
        self.ndof = 6

    def jacobian(self):
        """
        Define the jacobian for current dh.
        
        Creates the Jacobian for the gen3_lite dh table. Each column represents
        the cartesian movement of a point in relation to it's center about it's
        axis of rotation.
        """
        # step 1: compute the cumulative transformation
        T_cumulative = [np.eye(4)]
        for i, row in enumerate(self.dh):
            Ti = dh_to_h(row)
            T_cumulative.append(T_cumulative[-1] @ Ti)
        
        # step 3: compute difference in joint and EE position (r)
        J = np.zeros(3, self.ndof)
        for i in range(self.ndof):
            r = (T_cumulative[i] - T_cumulative[-1]) @ np.array([0, 0, 0, 1])
            z = T_cumulative[i][:3, :3] @ np.array([0, 0, 1])
            J[:, i] = np.cross(z, r)

        return J # the linear velo component of Jacobian
    
    def inv_jacobian(self, J, pseudo=False):
        """define the inverse jacobian for current dh"""

        if pseudo:
            return np.linalg.pinv(J)
        return np.linalg.inv(J)

    def damped_inv_jacobian(self, damping=0.25):
        """
        Define the damped inverse jacobian for current dh.

        Args:
            damping: the damping factor for the inverse jacobian
                set to .25 by default.

        Returns:
            The damped inverse jacobian formula A * A_inv
        """
        
        J = self.jacobian()

        JT = np.transpose(J)
        I = np.eye(3)
        return JT @ np.linalg.inv(J @ JT + (damping**2)*I)

    def position_fk(self):
        """find EE position given joint angles"""

    def velocity_fk(self):
        """move in cartesion coordinates by desired velocity"""

    def analytical_ik(self): 
        """analytically solve for the joint angles of a desired position"""

    def numerical_ik(self):
        """
        Numerically solve for the joint positions of a desired EE pose
        using the Newton-Raphson method of gradient descent.
        """

def dh_to_h(dh_i: list):
    al, a, d, t = dh_i[0], dh_i[1], dh_i[2], dh_i[3]
    T = np.array([
        [np.cos(t), -1 * np.cos(al) * np.sin(t), np.sin(al) * np.sin(t), a * np.cos(t)],
        [np.sin(t), np.cos(al) * np.cos(t), -1 * np.cos(t) * np.sin(al), a],
        [0, np.sin(al), np.cos(al), d],
        [0, 0, 0, 1]
    ])
    return T