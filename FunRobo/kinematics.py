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

    def jacobian(self):
        """
        Define the jacobian for current dh.
        
        Creates the Jacobian for the gen3_lite dh table. Each column represents
        the cartesian movement of a point in relation to it's center about it's
        axis of rotation.
        """
        
        
    def inv_jacobian(self):
        """define the inverse jacobian for current dh"""

    def damped_inv_jacobian(self):
        """define the damped inverse jacobian for current dh"""

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