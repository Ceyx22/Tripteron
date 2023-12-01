import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from util.GeneratorNode           import GeneratorNode
from util.TransformHelpers        import *
from tripteron.KinematicChain     import KinematicChain

# Import the format for the condition number message
from std_msgs.msg import Float64



class Trajectory():
    def __init__(self, node):
        # TODO: Rename the tips in the URDF to a better name
        self.fr_arm = KinematicChain(node, 'platform', 'theta3', self.jointnames())
        self.fl_arm = KinematicChain(node, 'platform', 'theta6', self.jointnames())
        self.br_arm = KinematicChain(node, 'platform', 'theta9', self.jointnames())
        self.bl_arm = KinematicChain(node, 'platform', 'theta10', self.jointnames())
        self.node = node

    # Declare the joint names.
    def jointnames(self, arm):
        if arm == 'frontleft':
            return ['theta1', 'theta2'] # , 'theta3'
        elif arm == 'frontright':
            return ['theta4', 'theta5'] # , 'theta6'
        elif arm == 'backleft':
            return ['theta7', 'theta8'] # , 'theta9'
        elif arm == 'backright':
            return ['theta10', 'theta11']# , 'theta12'
        
    
    def evaluate(self, t, dt):
        self.t += self.dt
        # Compute position/orientation of the pelvis (w.r.t. world).
        Pplatform = pxyz(0.0, 0.1, 0.1 + 0.2) # stay in one spot
        Rplatform = Rotz(0)
        Tplatform = T_from_Rp(Rplatform, Pplatform)
        # Build up and send the platform w.r.t. World Transform!
        trans = TransformStamped()
        trans.header.stamp    = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id  = 'platform'
        trans.transform       = Transform_from_T(Tplatform)
        self.broadcaster.sendTransform(trans)
        # Compute the joints.
        # q    = np.zeros((len(jointnames), 1))
        # qdot = np.zeros((len(jointnames), 1))

        # i_relbow = jointnames.index('r_arm_elx')

        # q[i_relbow,0]     = - pi/2 + pi/8 * sin(2*self.t)
        # qdot[i_relbow, 0] =          pi/4 * cos(2*self.t)

        # # Build up a command message and publish.
        # cmdmsg = JointState()
        # cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        # cmdmsg.name         = jointnames                # List of names
        # cmdmsg.position     = q.flatten().tolist()      # List of positions
        # cmdmsg.velocity     = qdot.flatten().tolist()   # List of velocities
        # self.pub.publish(cmdmsg)
        # fill in
        return None

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

