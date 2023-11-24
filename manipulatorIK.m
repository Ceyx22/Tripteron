clc;
clear all;
% tripteron
% addpath (genpath (strcat (pwd, ' \Dependencies')))
robot = importrobot('tripteron.urdf');
% robot = importrobot('sixDOF.urdf');
axes = show(robot);
axes.CameraPositionMode = 'auto';