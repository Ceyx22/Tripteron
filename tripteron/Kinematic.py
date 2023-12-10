'''Kinematic.py
'''

import enum
import rclpy
import numpy as np

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from std_msgs.msg               import String
from urdf_parser_py.urdf        import Robot

# Grab the utilities
from util.TransformHelpers   import *
from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp


#
#   Single Kinematic Step
#
#   This captures a single step from one frame to the next.  It be of type:
#
#     FIXED     Just a fixed T-matrix shift, nothing moving, not a DOF.
#     REVOLUTE  A fixed T-matrix shift, followed by a rotation about an axis.
#     LINEAR    A fixed T-matrix shift, followed by a transation along an axis.
#
#   A step contains several pieces of permanent data (coming from the URDF):
#
#     Tshift    Fixed shift: Transform of this frame w.r.t. previous
#     elocal    Joint axis (if applicable) in the local frame
#     type      One of the above
#     name      String showing the name
#     dof       If an active dof (not FIXED), the dof number
#
#   A step also contains several pieces of transient data, changing
#   over time or, more precisely, as the joint positions (angles) change:
#
#     T         Transform of this frame w.r.t. the world frame
#     p         Position  of this frame w.r.t. the world frame
#     R         Rotation  of this frame w.r.t. the world frame
#     e         Joint axis vector       w.r.t. the world frame
#
#   The latter information is reset and recomputed during each walk up
#   the chain.
#

#
#   Kinematic Chain Object
#
#   This stores the information provided by the URDF in the form of
#   steps (see above).  In particular, see the fkin() function, as it
#   walks up the chain to determine the transforms.
#

# Define the full kinematic chain
class kinematic():
    

    # Initialization.
    def __init__(self):
        # Store the node (for the helper functions).
        # self.node = node
        pltf_l = 0.1524
        pltf_w = 0.1524
        pltf_h = 0.1016
        slider_size = 0.1016
        limbRad = 0.0127
        self.toplimbLng = 0.1524
        self.bottomlimbLng = 0.2155  # 0.2155
        buffer = 0.0135

        self.O_x = (pltf_l / 2) - (limbRad + buffer)
        self.O_y = (pltf_w / 2) - (limbRad + buffer)
        self.O_z = pltf_h / 2

        # Grab the info from the URDF!
        # self.load(baseframe, tipframe, expectedjointnames)


    # def solve_kinematics(theta1, theta3, l1x, lb, lt, Px, Py, x, y):
    #     # Equation 1
    #     theta2 = theta1 + theta3

    #     # Equation 2
    #     l1y = x + Px + lt * np.cos(theta3) - l1x - lb * np.cos(theta1)

    #     # Equation 3
    #     l1 = y - Py - lt * np.sin(theta3) - l1y - lb * np.sin(theta1)

    #     return theta2, l1, l1y

    # def ikin(self, target):
    #     dist = np.linalg.norm(np.subtract(self.pos,target))
    #     max_distance = max(self.abs_length_m, min(self.total_limb_length, dist))

    #     # E_x = round(cos(diff_angle) * max_distance, 2)
    #     # E_y = round(sin(diff_angle) * max_distance, 2)
    #     # m = E_x ** 2 + E_y ** 2
    #     top_limb_sq = self.top_limb ** 2
    #     bottom_limb_sq = self.bottom_limb ** 2

    #     try:
    #         theta_1 = (acos((top_limb_sq - bottom_limb_sq + m) / (2 * self.top_limb * sqrt(m))) + diff_angle)
    #     except:
    #         print("This is not physically possible")
    #         return
    #     theta_2 = acos((top_limb_sq + bottom_limb_sq - m)/ (2 * self.top_limb * self.bottom_limb))



    def fkin(q):
        return None
        # # Temporary upper and lower limits
        # uLim_front = bottomlimbLng + toplimbLng
        # lLim_front = -(bottomlimbLng + toplimbLng)
        # uLim_back = bottomlimbLng + toplimbLng
        # lLim_back = -(bottomlimbLng + toplimbLng)
        # # Define the symbolic variables for the joint parameters and link lengths
        # # d, theta2, theta3, theta4, a1, a2, a3 = sp.symbols('d theta2 theta3 theta4 a1 a2 a3')

        # # Define the transformation matrix for the prismatic joint
        # T1 = T_from_Rp(np.eye(3), pxyz(d, 2, 0))

        # # Define the transformation matrices for the revolute joints
        # T2 = T_from_Rp(Rotx(theta2), pxyz(a1, 0, 0))
        # T3 = T_from_Rp(Roty(theta3), pxyz(a2, 0, 0))
        # T4 = T_from_Rp(Rotz(theta4), pxyz(a3, 0, 0))

        # # Combine the transformations to get the final transformation matrix
        # T_final = T1 @ T2 @ T3 @ T4

    def legOne (self,p):
        # chain from leg one to platform 
        #prist
        p_lowerArm_one = p
        R_lowerArm_one = Rotz(0)
        T_lowerArm_one_visual = T_from_Rp(R_lowerArm_one, p_lowerArm_one)

        # Transform for joint theta2 (lowerArm_one to upperArm_one)
        R_theta2 = R_from_URDF_rpy([pi/4, 0, 0])  # Inverse rotation
        p_theta2 = p_from_URDF_xyz([0, 0, self.toplimbLng])  # Inverse translation
        T_theta2 = T_from_Rp(R_theta2, p_theta2)

        # Transform for upperArm_one visual (centered in the middle of the cylinder)
        p_upperArm_one = p_from_URDF_xyz([0, 0, self.toplimbLng/2])
        R_upperArm_one = R_from_URDF_rpy([0, 0, 0])
        T_upperArm_one_visual = T_from_Rp(R_upperArm_one, p_upperArm_one)

        # Transform for joint theta1 (upperArm_one to platform)
        R_theta1 = R_from_URDF_rpy([pi/2, pi/4, 0])  # Inverse rotation
        p_theta1 = p_from_URDF_xyz([-self.O_x, self.O_y, self.O_z])  # Inverse translation
        T_theta1 = T_from_Rp(R_theta1, p_theta1)
        # Combine transformations to get the transform from the end-effector to the base
        T_base_from_end_effector = T_lowerArm_one_visual @ T_theta2 @ T_upperArm_one_visual @ T_theta1
        return T_base_from_end_effector
    
    # def legOneF (self,p):
    #     # chain from leg one to platform 
    #     #prist
    #     p_lowerArm_one = p
    #     R_lowerArm_one = Rotz(0)
    #     T_lowerArm_one_visual = T_from_Rp(R_lowerArm_one, p_lowerArm_one)

    #     # Transform for joint theta2 (lowerArm_one to upperArm_one)
    #     R_theta2 = R_from_URDF_rpy([pi/4, 0, 0])  # Inverse rotation
    #     p_theta2 = p_from_URDF_xyz([0, 0, self.toplimbLng])  # Inverse translation
    #     T_theta2 = T_from_Rp(R_theta2, p_theta2)

    #     # Transform for upperArm_one visual (centered in the middle of the cylinder)
    #     p_upperArm_one = p_from_URDF_xyz([0, 0, self.toplimbLng/2])
    #     R_upperArm_one = R_from_URDF_rpy([0, 0, 0])
    #     T_upperArm_one_visual = T_from_Rp(R_upperArm_one, p_upperArm_one)

    #     # Transform for joint theta1 (upperArm_one to platform)
    #     R_theta1 = R_from_URDF_rpy([pi/2, pi/4, 0])  # Inverse rotation
    #     p_theta1 = p_from_URDF_xyz([-self.O_x, self.O_y, self.O_z])  # Inverse translation
    #     T_theta1 = T_from_Rp(R_theta1, p_theta1)
    #     # Combine transformations to get the transform from the end-effector to the base
    #     T_base_from_end_effector = T_lowerArm_one_visual @ T_theta2 @ T_upperArm_one_visual @ T_theta1
    #     return T_base_from_end_effector

    # def IK_legOne():
    #     # Transform for lowerArm_one visual (centered in the middle of the cylinder)
    #     # This is just a translation along the z-axis
    #     p_lowerArm_one = p_from_URDF_xyz([0, 0, -bottomlimbLng/2])
    #     R_lowerArm_one = R_from_URDF_rpy([0, 0, 0])
    #     T_lowerArm_one_visual = T_from_Rp(R_lowerArm_one, p_lowerArm_one)

    #     # Transform for joint theta2 (lowerArm_one to upperArm_one)
    #     R_theta2 = R_from_URDF_rpy([pi/4, 0, 0])  # Inverse rotation
    #     p_theta2 = p_from_URDF_xyz([0, 0, toplimbLng])  # Inverse translation
    #     T_theta2 = T_from_Rp(R_theta2, p_theta2)

    #     # Transform for upperArm_one visual (centered in the middle of the cylinder)
    #     p_upperArm_one = p_from_URDF_xyz([0, 0, toplimbLng/2])
    #     R_upperArm_one = R_from_URDF_rpy([0, 0, 0])
    #     T_upperArm_one_visual = T_from_Rp(R_upperArm_one, p_upperArm_one)

    #     # Transform for joint theta1 (upperArm_one to platform)
    #     R_theta1 = R_from_URDF_rpy([pi/2, pi/4, 0])  # Inverse rotation
    #     p_theta1 = p_from_URDF_xyz([-O_x, O_y, O_z])  # Inverse translation
    #     T_theta1 = T_from_Rp(R_theta1, p_theta1)

    #     # Combine transformations to get the transform from the end-effector to the base
    #     T_base_from_end_effector = T_lowerArm_one_visual @ T_theta2 @ T_upperArm_one_visual @ T_theta1

    def compute_jacobian(x, y, z):
        J = np.zeros((3, 3))
    
        # Derivatives of f1, f2, f3 with respect to x, y, z
        # These should be replaced with the actual derivatives
        J[0, :] = [1, 1, 1]  # df1/dx, df1/dy, df1/dz
        J[1, :] = [1, -1, 1] # df2/dx, df2/dy, df2/dz
        J[2, :] = [1, 1, -1] # df3/dx, df3/dy, df3/dz
#
#   Main Code
#
#   This simply tests the kinematic chain and associated calculations!
#
def main(args=None):
    # Set the print options to something we can read.
    np.set_printoptions(precision=3, suppress=True)

    # Initialize ROS and the node.
    rclpy.init(args=args)
    node = Node('kintest')

    # Set up the kinematic chain object, assuming the 3 DOF.
    jointnames = ['theta1', 'theta2', 'theta3']
    baseframe  = 'world'
    tipframe   = 'tip'
    chain = KinematicChain(node, baseframe, tipframe, jointnames)

    # Define the test.
    def test(q):
        (ptip, Rtip, Jv, Jw) = chain.fkin(q)
        print('q:\n',       q)
        print('ptip(q):\n', ptip)
        print('Rtip(q):\n', Rtip)
        print('Jv(q):\n',   Jv)
        print('Jw(q):\n',   Jw)
        print('----------------------------------------');

    # Run the tests.
    test(np.radians(np.array([  20.0,   40.0,  -30.0])).reshape(3,1))
    test(np.radians(np.array([  30.0,   30.0,   60.0])).reshape(3,1))
    test(np.radians(np.array([ -45.0,   75.0,  120.0])).reshape(3,1))

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
