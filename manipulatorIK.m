clc;
clear all;
% tripteron
% addpath (genpath (strcat (pwd, ' \Dependencies')))
robot = importrobot('tripteron.urdf');
% robot = importrobot('sixDOF.urdf');
axes = show(robot);
% xlim([-5 5]);
% ylim([-5 5]);
% zlim([-5 5]);
axes.CameraPositionMode = 'auto';