clear
clc

% robot_arm URDF
addpath(genpath('urdf'),genpath('Utilities'))
robot_arm1 = importrobot('manipulator1.urdf');
robot_arm2 = importrobot('manipulator2.urdf');

% �������ݸ�ʽ
robot_arm1.DataFormat = 'column';
robot_arm2.DataFormat = 'column';

homeRobotConfig1= homeConfiguration(robot_arm1);
homeRobotConfig2 = homeConfiguration(robot_arm2);
randomRobotConfig=robot_arm1.randomConfiguration;
targetRobotConfig1=[1.1,0.5,2.1,-0.3,1.7]';
targetRobotConfig2=[0,1.8,0,1,1]';

% t=getTransform(robot_arm1,targetRobotConfig2,'end_effector_link');
% s=tform2trvec(t)
% 
% 
% show(robot_arm1,targetRobotConfig2,'Frames','off');


collisionArrayFromMesh=cell(robot_arm1.NumBodies,2);

robotarm_Collision = importrobot('manipulator1.urdf');
robotarm_Collision.DataFormat = 'column';

% ȡ����Ӧ�ؽڵ� STL �ļ�������е��ĩ�˳���
robotBodies = [{robotarm_Collision.Base} robotarm_Collision.Bodies];%%�����������ݱ�ʾ����е�ۻ�����������
for i = 1:numel(robotBodies)-1  %������е��ĩ�˳���
    if ~isempty(robotBodies{i}.Visuals)
        % Assumes the first Visuals element is the correct one.
        visualDetails = robotBodies{i}.Visuals{1};
        
        % Extract the part of the visual that actually specifies the STL name
        visualParts = strsplit(visualDetails, ':');
        stlFileName = visualParts{2};
        
        % Read the STL file
        stlData = stlread(stlFileName);
        
        % Create a collisionMesh object from the vertices
        collisionArrayFromMesh{i,1} = collisionMesh(stlData.Points);
        
        % Transform is always identity
        collisionArrayFromMesh{i,2} = eye(4);
    end
end

config = [-pi/2 4*pi/5 pi/6 0 0 ]';
% config=[pi/2,pi/2,pi/2,pi/2,pi/2]';
[isCollision, selfCollisionPairIdx] = manipCheckCollisions(robotarm_Collision, collisionArrayFromMesh, {}, config, true);
disp(isCollision)

show(robotarm_Collision, config,'Frames','on');

highlightCollisionBodies(robotarm_Collision, selfCollisionPairIdx, gca);%gca��ʾ��ǰ�����򿪵�����ϵ