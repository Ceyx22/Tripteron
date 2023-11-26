clc;
clear all;

robot = importrobot(['D:\HW\Fall Smore\ME_133a\Project\Description\tripteronBodyFull.xacro']);
% robot = importrobot(['D:\HW\Fall Smore\ME_133a\Project\Description\tripteronBody.xacro']);

axes = show(robot);
axes.CameraPositionMode = 'auto';
