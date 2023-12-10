'''tripmove.py

   This is a demo for moving/placing an ungrounded robot and moving joints.

   Node:        /tripmove
   Publish:     /joint_states                 sensor_msgs/JointState
   Broadcast:   'platform' w.r.t. 'world'     geometry_msgs/TransformStamped

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState

from tripteron.KinematicChain     import KinematicChain
from util.TransformHelpers     import *


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

         # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass
        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))
        self.fr_arm = KinematicChain(self, 'platform', 'railAttach_one', self.jointnames("frontright"))
        self.qfr = np.zeros((3,1))
        self.qdotfr= np.zeros((3,1))

    # Shutdown.
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()        
    # Declare the joint names.
    def jointnames(self, arm):
        # ask gunter about wether the last joint should be fixed
        if arm == 'frontright':
            return ['theta1', 'theta2', 'theta3'] # , 'theta3'
        elif arm == 'frontleft':
            return ['theta4', 'theta5', 'theta6'] # , 'theta6'
        elif arm == 'backright':
            return ['theta7', 'theta8', 'theta9'] # , 'theta9'
        elif arm == 'backleft':
            return ['theta10', 'theta11', 'theta12']# , 'theta12'
        
        # return ['theta1', 'theta2','theta4', 'theta5', 
                # 'theta7', 'theta8', 'theta10', 'theta11']
        # return ['theta1', 'theta2', 'theta3','theta4', 'theta5', 'theta6', 
        #         'theta7', 'theta8', 'theta9', 'theta10', 'theta11', 'theta12']


    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)

    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt
        # Compute position/orientation of the pelvis (w.r.t. world).
        Pplatform = pxyz(0.1*sin(self.t), 0.1*cos(self.t), 0.1 + 0.2) # stay in one spot
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
        # q    = self.qfr
        # qdot = self.qdotfr
        q    = np.zeros((len(self.jointnames("frontright")), 1))
        qdot = np.zeros((len(self.jointnames("frontright")), 1))

        q[0,0]     = - pi/2 + pi/8 * sin(2*self.t)
        qdot[0,0] =          pi/4 * cos(2*self.t)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = self.jointnames("frontright")     # List of names
        cmdmsg.position     = q.flatten().tolist()      # List of positions
        cmdmsg.velocity     = qdot.flatten().tolist()   # List of velocities
        self.pub.publish(cmdmsg)

        
#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    node = DemoNode('tripmoveadv', 100)

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
