clear
clc

addpath(genpath('urdf'),genpath('Utilities'))
robot_arm1 = importrobot('collision2.urdf');
robot_arm2 = importrobot('manipulator2.urdf');
robot_arm1.DataFormat = 'column';
robot_arm2.DataFormat = 'column';

targetRobotConfig1=[pi/2,pi/2,pi/2,pi/2,pi/2]';
randomRobotConfig=robot_arm1.randomConfiguration;
homeRobotConfig1= homeConfiguration(robot_arm1);

figure
show(robot_arm1,targetRobotConfig1,'Frames','off');
figure
show(robot_arm2,targetRobotConfig1,'Frames','off');