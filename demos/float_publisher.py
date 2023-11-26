"""float_publisher.py

   Publish a floating point number as set by a GUI slider.

   Node:        /float
   Publish:     /float                  std_msgs/Float64

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
from std_msgs.msg       import Float64


#
#  GUI Slider Class
#
class VarGUI(QWidget):
    def __init__(self, val, callback):
        super().__init__()
        name   = 'Data'
        minval = -1.0
        maxval =  1.0
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
        self.setWindowTitle('Data')
        self.show()

    def valueHandler(self, value):
        self.value = self.offset + self.slope * float(value)
        self.number.setText("%6.3f" % self.value)
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
        self.pub = self.create_publisher(Float64, '/float', quality)

        # Save the publisher rate and status.
        self.publishrate = rate
        self.publishing  = False

        # Create the float message.
        self.float = Float64()
        self.setvalue(initvalue)


    # Run
    def run(self):
        # Prepare Qt.
        app = QApplication(sys.argv)
        app.setApplicationDisplayName("Float Publisher")

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
        self.pub.publish(self.float)

    # Get/Set the value.
    def getvalue(self):
        # Get the value.
        return self.float.data
    def setvalue(self, value):
        # Set the value.
        self.float.data = value


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the GUI node (10Hz).
    rclpy.init(args=args)
    node = GUINode('float', 0.0, 10)

    # Run until interrupted.
    node.run()

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
