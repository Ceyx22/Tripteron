"""Launch the interactive marker demo

   ros2 launch demos interactivedemo.launch.py

   This is only intended to demonstrate the example.  Please
   edit/update as appropriate.

   This should start
     1) RVIZ, configured to see the visualization markers
     2) The interactive marker demo

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('demos'), 'rviz/viewmarkers.rviz')


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the point_publisher.
    node_demo = Node(
        name       = 'interactivedemo',
        package    = 'demos',
        executable = 'interactivedemo',
        output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())


    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        # Start the demo and RVIZ
        node_demo,
        node_rviz,
    ])
