clc;
clear all;

robot = importrobot(['/home/ceyx/Documents/tripteron/robotDescription/tripteronBody.urdf.xacro']);
% robot = importrobot(['D:\HW\Fall Smore\ME_133a\Project\Description\tripteronBody.xacro']);

axes = show(robot);
axes.CameraPositionMode = 'auto';
