"""point_publisher.py

   Publish an x/y/z point, as set by three GUI sliders.

   This generates a point message, as well as a visualization marker
   array, which rviz can render.

   Node:        /point
   Publish:     /point                          std_msgs/Point64
   Publish:     /visualization_marker_array     visualization_msgs/MarkerArray

"""

import rclpy
import numpy as np
import signal
import sys
import threading

from PyQt5.QtCore           import (Qt, QTimer)
from PyQt5.QtWidgets        import (QApplication,
                                    QWidget, QLabel, QHBoxLayout, QVBoxLayout,
                                    QSlider, QCheckBox)

from rclpy.node             import Node
from rclpy.qos              import QoSProfile, DurabilityPolicy
from geometry_msgs.msg      import Point
from geometry_msgs.msg      import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
#  GUI Slider Class
#
class SingleVariable(QWidget):
    def __init__(self, name, val, minval, maxval, callback):
        super().__init__()
        self.value    = val
        self.offset   = (maxval + minval) / 2.0
        self.slope    = (maxval - minval) / 200.0
        self.callback = callback
        self.initUI(name)

    def initUI(self, name):
        # Top-Left: Name
        label = QLabel(name)
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setMinimumWidth(40)

        # Top-Right: Number
        self.number = QLabel("%6.3f" % self.value)
        self.number.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.number.setMinimumWidth(100)
        self.number.setStyleSheet("border: 1px solid black;")

        # Bottom: Slider
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-100, 100)
        slider.setFocusPolicy(Qt.NoFocus)
        slider.setPageStep(5)
        slider.valueChanged.connect(self.valueHandler)
        slider.setValue(int((self.value - self.offset)/self.slope))

        # Create the Layout
        hbox = QHBoxLayout()
        hbox.addWidget(label)
        hbox.addSpacing(10)
        hbox.addWidget(self.number)

        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(slider)

        self.setLayout(vbox)

    def valueHandler(self, value):
        self.value = self.offset + self.slope * float(value)
        self.number.setText("%6.3f" % self.value)
        self.callback(self.value)

    
class XYZGUI(QWidget):
    def __init__(self, p0, callback):
        super().__init__()
        self.value    = p0
        self.callback = callback
        self.initUI(p0)

    def initUI(self, p):
        # Create the XYZ variables
        vbox = QVBoxLayout()
        vbox.addWidget(SingleVariable('X', p[0], -1.0, 1.0, self.xHandler))
        vbox.addWidget(SingleVariable('Y', p[1], -1.0, 1.0, self.yHandler))
        vbox.addWidget(SingleVariable('Z', p[2],  0.0, 2.0, self.zHandler))

        self.setLayout(vbox)
        self.setWindowTitle('XYZ Position')
        self.show()

    def xHandler(self, value):
        self.value[0] = value
        self.callback(self.value)

    def yHandler(self, value):
        self.value[1] = value
        self.callback(self.value)

    def zHandler(self, value):
        self.value[2] = value
        self.callback(self.value)

    def kill(self, signum, frame):
        self.close()


#
#   GUI Node Class
#
class GUINode(Node):
    # Initialization.
    def __init__(self, name, initvalue, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pub_mark  = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)
        self.pub_point = self.create_publisher(
            PointStamped, '/point', quality)

        # Save the publisher rate and status.
        self.publishrate = rate
        self.publishing  = False

        # Create the point.
        self.p = Point()
        self.setvalue(initvalue)

        # Create the point message.
        self.point = PointStamped()
        self.point.header.frame_id = "world"
        self.point.header.stamp    = self.get_clock().now().to_msg()
        self.point.point           = self.p

        # Create the sphere marker.
        self.marker = Marker()
        self.marker.header.frame_id    = "world"
        self.marker.header.stamp       = self.get_clock().now().to_msg()

        self.marker.action             = Marker.ADD
        self.marker.ns                 = "point"
        self.marker.id                 = 1
        self.marker.type               = Marker.SPHERE
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position      = self.p
        self.marker.scale.x            = 0.2
        self.marker.scale.y            = 0.2
        self.marker.scale.z            = 0.2
        self.marker.color.r            = 1.0
        self.marker.color.g            = 0.0
        self.marker.color.b            = 0.0
        self.marker.color.a            = 0.8     # Make transparent!

        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)


    # Run
    def run(self):
        # Prepare Qt.
        app = QApplication(sys.argv)
        app.setApplicationDisplayName("Point Publisher")

        # Include a Qt Timer, setup up so every 500ms the python
        # interpreter runs (doing nothing).  This enables the ctrl-c
        # handler to be processed and close the window.
        timer = QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)

        # Then setup the GUI window.  And declare the ctrl-c handler to
        # close that window.
        gui = XYZGUI(self.getvalue(), self.setvalue)
        signal.signal(signal.SIGINT, gui.kill)

        # Start the publisher in a separate thread.
        self.publishing = True
        thread = threading.Thread(target=self.publisher)
        thread.start()

        # Start the GUI window.
        self.get_logger().info("GUI starting...")
        status = app.exec_()
        self.get_logger().info("GUI ended.")

        # End the publisher.
        self.publishing = False
        thread.join()


    # Publisher Loop
    def publisher(self):
        # Create a timer to control the publishing.
        rate  = self.publishrate
        timer = self.create_timer(1/float(rate), self.publish)
        dt    = timer.timer_period_ns * 1e-9
        self.get_logger().info("Publishing with dt of %f seconds (%fHz)" %
                               (dt, rate))

        # Publish until told to stop.
        while self.publishing:
            rclpy.spin_once(self)

        # Destroy the timer and report.
        timer.destroy()        
        self.get_logger().info("Stopped publishing")


    # Publish the current value.
    def publish(self):
        # Grab the current time.
        now = self.get_clock().now()
        # Publish.
        self.marker.header.stamp = now.to_msg()
        self.point.header.stamp  = now.to_msg()
        self.pub_mark.publish(self.mark)
        self.pub_point.publish(self.point)


    # Get/Set the value.
    def getvalue(self):
        # Get the value.
        return [self.p.x, self.p.y, self.p.z]
    def setvalue(self, value):
        # Set the value.
        self.p.x = value[0]
        self.p.y = value[1]
        self.p.z = value[2]


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the GUI node (10Hz).
    rclpy.init(args=args)
    node = GUINode('point', [0.5, 0.5, 0.5], 10)

    # Run until interrupted.
    node.run()

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
