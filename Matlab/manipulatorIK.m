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

     