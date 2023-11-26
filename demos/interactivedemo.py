"""intereactivedemo.py

   Demostrate the use of interactive markers.

   Node:        /interactivedemo

"""

import rclpy
import numpy as np

from math import pi, sin, cos, asin, acos, atan2, sqrt, fmod, exp

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray
from visualization_msgs.msg     import InteractiveMarker
from visualization_msgs.msg     import InteractiveMarkerControl

from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from demos.TransformHelpers     import *


#
#  Interactive XZ-Ring Marker Server
#
#  Create an interactive marker, forming a ring parallel to the XZ
#  plane and moving in its plane.
#
#  There are simpler markers, but I wanted a ring.  So I had to use a
#  "LINE_STRIP" which is just a sequence of concatenated points...
#
class XZRingMarker:
    def __init__(self, node, p, r):
        # Store the reference to the node and the point p.
        self.node = node
        self.p    = p

        # Create a marker of type LINE_STRIP for the ring.
        marker = Marker()
        marker.header.frame_id  = "world"
        marker.header.stamp     = node.get_clock().now().to_msg()
        marker.action           = Marker.ADD
        marker.ns               = "ring"
        marker.id               = 0
        marker.type             = Marker.LINE_STRIP
        marker.pose.orientation = Quaternion()
        marker.pose.position    = Point_from_p(self.p) # Center
        marker.scale.x          = 0.2 * r  # Linewidth
        marker.color            = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0) # Yellow

        # Add the list of points to make the ring around the center.
        N = 32
        for i in range(N+1):
            theta = 2 * pi *  float(i) / float(N)
            marker.points.append(Point(x = r*sin(theta), y = 0.0, z = r*cos(theta)))

        # Create an interactive marker for our server.  Set the
        # position to coincide with the ring, and scale so the handles
        # reach outside the ring.
        imarker = InteractiveMarker()
        imarker.header.frame_id = "world"
        imarker.name            = "ring"
        imarker.pose.position   = Point_from_p(self.p)
        imarker.scale           = 2.0 * r

        # Append a non-interactive control which contains the ring.
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        imarker.controls.append(control)

        # Append an interactive control for X movement
        # The x axis is the main move-axis.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name             = "move_x"
        control.orientation      = Quaternion()
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)

        # Append an interactive control for Z movement.
        # Rotate the main move-axis (x) by 90 deg about y.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name             = "move_z"
        control.orientation      = Quaternion_from_R(Roty(pi/2))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)
        
        # Create an interactive marker server on the topic namespace
        # ring, setup the feedback, and apply/send to all clients.
        server = InteractiveMarkerServer(node, 'ring')
        server.insert(imarker, feedback_callback=self.process)
        server.applyChanges()

        # Report.
        node.get_logger().info("Ring Interactive Marker and Subscriber set up...")

    def position(self):
        # Return the point.
        return self.p

    def process(self, msg):
        # Save the point contained in the message.
        self.p = p_from_Point(msg.pose.position)

        # Report (to show working)
        self.node.get_logger().info("Ring moved to: " + str(self.p.T))


#
#  Interactive Vector Marker Server
#
#  Create an interactive marker, forming a fixed-length vector and
#  rotating like a pan/tilt gimbal.  Note the vector is the x-axis in
#  the rotated frame.
#
class PanTiltVectorMarker:
    def __init__(self, node, p, d, l):
        # Store the reference to the node.
        self.node  = node

        # Make sure the normal direction vector is normalized.
        self.n = d / np.linalg.norm(d)

        # Convert the direction into a quaternion (via pan/tilt).
        tilt = asin(self.n[2,0])
        pan  = atan2(self.n[1,0], self.n[0,0])

        # Create a marker of type ARROW for the vector, unrotated.
        marker = Marker()
        marker.header.frame_id  = "world"
        marker.header.stamp     = node.get_clock().now().to_msg()
        marker.action           = Marker.ADD
        marker.ns               = "vector"
        marker.id               = 0
        marker.type             = Marker.ARROW
        marker.pose.orientation = Quaternion_from_R(Rotz(pan)@Roty(-tilt))
        marker.pose.position    = Point_from_p(p)
        marker.scale            = Vector3(x=l, y=0.05*l, z=0.05*l)
        marker.color            = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) # Cyan
        # Scale = Length x Width x Arrow Height

        # Create an interactive marker for our server at the position
        # and with the initial orientation.
        imarker = InteractiveMarker()
        imarker.header.frame_id = "world"
        imarker.name = "vector"
        imarker.pose.orientation = Quaternion_from_R(Rotz(pan)@Roty(-tilt))
        imarker.pose.position    = Point_from_p(p)
        imarker.scale            = l

        # Append a non-interactive control which contains the vector.
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        imarker.controls.append(control)

        # Append an interactive control for Z rotation: pan.
        # Get Z-axis, rotating the main (x-axis) for 90 deg about y.
        # Fixed means the control axis is given in world frame.
        control = InteractiveMarkerControl()
        control.name             = "rotate_pan"
        control.orientation      = Quaternion_from_R(Roty(pi/2))
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)

        # Append an interactive control for Y rotation.
        # Get Y-axis, rotating the main (x-axis) for 90 deg about z.
        # Inheriting means the control axis is given in the reoriented frame.
        control = InteractiveMarkerControl()
        control.name             = "rotate_tilt"
        control.orientation      = Quaternion_from_R(Rotz(pi/2))
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.INHERIT
        imarker.controls.append(control)

        # Create an interactive marker server on the topic namespace
        # ring, setup the feedback, and apply/send to all clients.
        server = InteractiveMarkerServer(node, 'vector')
        server.insert(imarker, feedback_callback=self.process)
        server.applyChanges()

        # Report.
        node.get_logger().info("Vector Interactive Marker and Subscriber set up...")

    def vector(self):
        # Return the point.
        return self.n

    def process(self, msg):
        # Save the direction contained in the message.  Find the
        # x-axis in the rotated frame (given by the quaternions).
        self.n = R_from_Quaternion(msg.pose.orientation) @ ex()

        # Report (to show working)
        self.node.get_logger().info("Vector rotated to: " + str(self.n.T))


#
#  Interactive Point Marker Server
#
#  Create an interactive marker, forming a small sphere (point) and
#  moving in 3D directions.
#
class PointMarker:
    def __init__(self, node, p, r):
        # Store the reference to the node.
        self.node  = node

        # Create a marker of type SHPERE for the point.
        marker = Marker()
        marker.header.frame_id  = "world"
        marker.header.stamp     = node.get_clock().now().to_msg()
        marker.ns               = "point"
        marker.id               = 0
        marker.type             = Marker.SPHERE
        marker.action           = Marker.ADD
        marker.pose.orientation = Quaternion()
        marker.pose.position    = Point_from_p(p) # Center
        marker.scale            = Vector3(x = 2*r, y = 2*r, z = 2*r)
        marker.color            = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0) # Magneta

        # Create an interactive marker for our server with the initial
        # position, and scale so the handles reach outside the sphere.
        imarker = InteractiveMarker()
        imarker.header.frame_id = "world"
        imarker.name = "point"
        imarker.pose.position   = Point_from_p(p)
        imarker.scale           = 2*r

        # Append a non-interactive control which contains the sphere.
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        imarker.controls.append(control)

        # Append an interactive control for X movement
        # The x axis is the main move-axis.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name             = "move_x"
        control.orientation      = Quaternion()
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)

        # Append an interactive control for Y movement.
        # Get Y-axis, rotating the main (x-axis) for 90 deg about z.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.orientation      = Quaternion_from_R(Rotz(pi/2))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)
        
        # Append an interactive control for Z movement.
        # Get Z-axis, rotating the main (x-axis) for 90 deg about y.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.orientation      = Quaternion_from_R(Roty(pi/2))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)
        
        # Create an interactive marker server on the topic namespace
        # point, setup the feedback, and apply/send to all clients.
        server = InteractiveMarkerServer(node, 'point')
        server.insert(imarker, feedback_callback=self.process)
        server.applyChanges()

        # Report.
        node.get_logger().info("Point Interactive Marker and Subscriber set up...")

    def position(self):
        # Return the point.
        return self.p

    def process(self, msg):
        # Save the point contained in the message.
        self.p = p_from_Point(msg.pose.position)

        # Report (to show working)
        self.node.get_logger().info("Point moved to: " + str(self.p.T))


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Report.
        self.get_logger().info("Starting the demo for interactive markers...")
        
        # Create the interactive ring marker server.
        p = np.array([0.2, 0.0, 0.6]).reshape((3,1))
        ring = XZRingMarker(self, p, 0.1)

        # Create the interactive vector marker server.
        p = np.array([0.0, 0.3,   0.0  ]).reshape((3,1))
        d = np.array([0.0, 0.707, 0.707]).reshape((3,1))
        vector = PanTiltVectorMarker(self, p, d, 0.4)

        # Create the interactive point marker server.
        p = np.array([-0.2, 0.3, 0.6]).reshape((3,1))
        point = PointMarker(self, p, 0.05)


#
#  Main Code
#
def main(args=None):
    # Set the numpy printing options (as not to be overly confusing).
    # This is entirely to make it look pretty (in my opinion).
    np.set_printoptions(suppress = True, precision = 6)

    # Initialize ROS and the demo node.
    rclpy.init(args=args)
    node = DemoNode('interactivedemo')

    # Run until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
