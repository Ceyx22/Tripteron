"""boolean_publisher.py

   Publish a boolean topic as set by a GUI window.

   Node:        /boolean
   Publish:     /boolean                std_msgs/Bool

"""

import rclpy
import numpy as np
import signal
import sys
import threading

from PyQt5.QtCore       import (Qt, QTimer)
from PyQt5.QtWidgets    import (QApplication,
                                QWidget, QLabel, QHBoxLayout, QVBoxLayout,
                                QSlider, QCheckBox)

from rclpy.node         import Node
from rclpy.qos          import QoSProfile, DurabilityPolicy
from std_msgs.msg       import Bool


#
#  GUI Checkbox Class
#
class VarGUI(QWidget):
    def __init__(self, b, callback):
        super().__init__()
        name          = 'On/Off'
        self.callback = callback
        self.initUI(name, b)

    def initUI(self, name, b):
        # Checkbox
        self.checkbox = QCheckBox(name)
        self.checkbox.stateChanged.connect(self.valueHandler)
        self.checkbox.setChecked(b)
     
        # Create the Layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.checkbox)

        self.setLayout(vbox)
        self.setWindowTitle('Data')
        self.show()

    def valueHandler(self):
        self.callback(self.checkbox.isChecked())

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
        self.pub = self.create_publisher(Bool, '/boolean', quality)

        # Save the publisher rate and status.
        self.publishrate = rate
        self.publishing  = False

        # Create the boolean message.
        self.boolean = Bool()
        self.setvalue(initvalue)


    # Run
    def run(self):
        # Prepare Qt.
        app = QApplication(sys.argv)
        app.setApplicationDisplayName("Boolean Publisher")

        # Include a Qt Timer, setup up so every 500ms the python
        # interpreter runs (doing nothing).  This enables the ctrl-c
        # handler to be processed and close the window.
        timer = QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)

        # Then setup the GUI window.  And declare the ctrl-c handler to
        # close that window.
        gui = VarGUI(self.getvalue(), self.setvalue)
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
        # Publish
        self.pub.publish(self.boolean)

    # Get/Set the value.
    def getvalue(self):
        # Get the value.
        return self.boolean.data
    def setvalue(self, b):
        # Set the value.
        self.boolean.data = b


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the GUI node (10Hz).
    rclpy.init(args=args)
    node = GUINode('boolean', True, 10)

    # Run until interrupted.
    node.run()

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
