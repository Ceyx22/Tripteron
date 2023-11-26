clc;
clear all;
% tripteron
% addpath (genpath (strcat (pwd, ' \Dependencies')))
robot = importrobot(['Project\URDF\tripteronBody.urdf']);
% robot = importrobot(['Project\URDF\exmple.urdf']);

% robot = importrobot('sixDOF.urdf');
axes = show(robot);
% xlim([-5 5]);
% ylim([-5 5]);
% zlim([-5 5]);
axes.CameraPositionMode = 'auto';

% norm([0.2, -0.2, 0.2])

% rpy="-0.7854 -0.7854 -2.3562"

     <!-- Base_link to Upper arm  -->
     <joint name="theta1" type="continuous">
          <parent link="base_link"/>
          <child  link="upperArm_one"/>
          <origin xyz="0.0500 -0.0500 -0.0500" rpy="0 0 0"/>  
          <axis   xyz="0 0 1"/>
     </joint>
     
     <link name="upperArm_one">
          <visual>
               <origin xyz="0.0373 -0.0538 -0.0411" rpy="-0.7854 -0.7854 0"/>	 
               <geometry>
                    <cylinder radius="0.0127" length="0.1524"/> 
               </geometry>
               <material name="cyan"/>
          </visual>
     </link>