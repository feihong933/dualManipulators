function importRobotModel
% Imports robot model into a Rigid Body Tree

    % robot_arm URDF
    addpath(genpath('urdf'))
    robot_arm1 = importrobot('robot_arm.urdf');
    robot_arm2= importrobot('robot_arm01.urdf');
    % Add a "dummy" gripper link
 
    eeOffset = 0.06;
    gripperBody = robotics.RigidBody('Gripper');
    gripperBody.Mass = 1;
    gripperBody.Inertia = [0 0 0 0 0 0];
    setFixedTransform(gripperBody.Joint,trvec2tform([0 0.03 eeOffset]));
%     添加机械臂末端
    addBody(robot_arm1,gripperBody,'link6');
    addBody(robot_arm2,gripperBody,'link6');
    robot_arm1.Gravity=[0 0 -9.81];
    
%     gripperLength = 0.1; % Gripper length in meters
%     gripperBody = robotics.RigidBody('Gripper');
%     gripperJoint = robotics.Joint('GripperLink','fixed');
%     T = rotm2tform([0 1 0;0 0 1;1 0 0]) * trvec2tform([gripperLength 0 0]);
%     setFixedTransform(gripperJoint,T); % Move and orient the gripper
%     gripperBody.Joint = gripperJoint;
%     addBody(gen3,gripperBody,'EndEffector_Link');

% 设置数据格式
     robot_arm1.DataFormat = 'row';
     robot_arm2.DataFormat = 'row';
     
% % Save the Rigid Body Tree to a file
%     curDir = pwd;
%     saveDir = fileparts(mfilename('fullpath'));
%     cd(saveDir)
%     save robot_arm1 robot_arm1
%     save robot_arm2 robot_arm2
%     cd(curDir)
homeRobotConfig = homeConfiguration(robot_arm1);
homeRobotConfig = homeConfiguration(robot_arm2);
% randomRobotConfig=robot_arm1.randomConfiguration
targetRobotConfig1=[2,2,2,2,2];
targetRobotConfig2=[pi/2,pi/2,0,pi/2,pi/2];
% showdetails(robot_arm);
% getTransform(robot_arm1,targetRobotConfig,'link5')


tform=getTransform(robot_arm1,homeConfiguration(robot_arm1),'Gripper','base_link')


%%逆解
ik=robotics.InverseKinematics('RigidBodyTree',robot_arm1);
weights= [0.25 0.25 0.25 1 1 1];
initialguess=homeRobotConfig;
ik.SolverParameters.AllowRandomRestarts = true;
% ik.SolverParameters.MaxIterations = 1500;
[config,info]=ik('Gripper',tform,weights,initialguess);
config


figure
show(robot_arm1,targetRobotConfig1 ,'Frames','off');
hold on
show(robot_arm2,targetRobotConfig1,'Frames','off');

view([1024 960]);%figure大小设置
axis([-0.5 0.8 -0.5 0.5 0 0.5]);%坐标范围
camva('auto');%设置相机视角




