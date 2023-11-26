"""Launch the float publisher demo

   ros2 launch demos float_publisher.launch.py

   This is only intended to demonstrate the example.  Please
   edit/update as appropriate.

   This starts:
   1) The float_publisher.py
   2) A "ros2 topic echo /float" commmand - definitely only for demo!

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import ExecuteProcess
from launch.actions              import RegisterEventHandler
from launch.actions              import Shutdown
from launch.event_handlers       import OnProcessExit
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the float_publisher.
    node_publisher = Node(
        name       = 'float',
        package    = 'demos',
        executable = 'float_publisher',
        output     = 'screen',
        on_exit    = Shutdown())

    # Run the ros2 topic echo command:
    sleep_before_echo = ExecuteProcess(
        cmd = ['sleep', '2'])

    cmd_echo = ExecuteProcess(
        cmd    = ['ros2', 'topic', 'echo', '/float'],
        output = 'screen')

    delay_echo = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = sleep_before_echo,
            on_exit       = [cmd_echo]))



    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([

        # Register the delayed events first.
        delay_echo,

        # Start the float pulisher and the topic echo.
        node_publisher,
        sleep_before_echo,
    ])
