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
        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

    # Shutdown.
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()        

    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)
    
    def jointnames(self):
        # ask gunter about wether the last joint should be fixed
        
        return ['right_slider_1', 'right_theta_1', 'right_theta_2'] # , 'theta3'

        
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        q    = np.zeros((3, 1))
        qdot = np.zeros((3, 1))

        q[0,0]     = - pi/2 + pi/8 * sin(2*self.t)
        qdot[0,0] =          pi/4 * cos(2*self.t)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = self.jointnames()         # List of names
        cmdmsg.position     = q.flatten().tolist()      # List of positions
        cmdmsg.velocity     = qdot.flatten().tolist()   # List of velocities
        self.pub.publish(cmdmsg)

        
#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    node = DemoNode('tripmove', 100)

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
