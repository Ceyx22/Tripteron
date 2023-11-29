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

from tripteron.TransformHelpers     import *


#
#   Tripteron Joint Names
#
# jointnames = ['theta1', 'theta2', 'theta3',
#               'theta4', 'theta5', 'theta6',
#               'theta7', 'theta8', 'theta9',
#               'theta10', 'theta11', 'theta12']


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

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown.
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()

    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)

    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt
        # Compute position/orientation of the pelvis (w.r.t. world).
        Pplatform = pxyz(0.0, 0.1, 0.1 + 0.2 * sin(self.t/2))
        Rplatform = Rotz(sin(self.t))
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
