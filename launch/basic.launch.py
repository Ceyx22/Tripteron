""" Launch the View publisher demo

    ros2 launch tripteron basic.launch.py

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import ExecuteProcess
from launch.actions              import RegisterEventHandler
from launch.actions              import Shutdown
from launch.event_handlers       import OnProcessExit
from launch_ros.actions          import Node
import xacro

#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES
    pkg_name = 'tripteron'
    file_subpath = 'urdf/tripteronBodyFull.urdf.xacro'
    xacro_file = os.path.join(pkgdir(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    rvizcfg = os.path.join(pkgdir(pkg_name), 'rviz/viewrobot.rviz')

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description_raw}])
    
    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    # Configure a node for the move demo.
    node_trajectory = Node(
        name       = 'trajectory', 
        package    = 'tripteron',
        executable = 'basic',
        output     = 'screen')

    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        # Register the delayed events first.
        node_robot_state_publisher,
        node_rviz,
        node_trajectory,

    ])
