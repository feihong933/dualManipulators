clear
clc
%% �汾˵����

%ȥ���ؽ��˶�ģ�ͣ�ֱ��ʹ�����ζ���ʽ�켣�滮�������2019.12.25

%�Ľ���ײ���ģ�ͣ������Բ���򻯡���2020.01.03

%% robot_arm URDF
addpath(genpath('urdf'),genpath('Utilities'))
robot_arm1 = importrobot('manipulator1.urdf');
robot_arm2 = importrobot('manipulator2.urdf');

% �������ݸ�ʽ
robot_arm1.DataFormat = 'column';
robot_arm2.DataFormat = 'column';
        
homeRobotConfig1= homeConfiguration(robot_arm1);
homeRobotConfig2 = homeConfiguration(robot_arm2);
randomRobotConfig=robot_arm1.randomConfiguration;
targetRobotConfig1=[pi/2,pi/2,pi/2,pi/2,pi/2];
targetRobotConfig2=[pi/2,pi/2,0,pi/2,pi/2];

% showdetails(robot_arm);
% getTransform(robot_arm1,targetRobotConfig,'link5')

currentRobotJConfig = homeConfiguration(robot_arm1);
numJoints = numel(currentRobotJConfig);
endEffector = "end_effector_link";

timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

%% ��ʼ��Ȳ�������
jointInit = currentRobotJConfig;
taskInit1 = getTransform(robot_arm1,jointInit,endEffector);
taskInit2 = getTransform(robot_arm2,jointInit,endEffector);
% taskFinal = trvec2tform([0.3,0.1,0.25])*axang2tform([1 0 0 -pi/2])
taskFinal = trvec2tform([0.32,0.05,0.25]);
taskFinal_ = trvec2tform([0.1,0,0.35]);
display(taskFinal)
display(taskFinal_)

%compute tool traveling distance
distance1 = norm(tform2trvec(taskInit1)-tform2trvec(taskFinal));
distance2 = norm(tform2trvec(taskInit2)-tform2trvec(taskFinal_));
%define trajectory times based on traveling distance and desired tool speed
initTime = 0;
finalTime1 = (distance1/toolSpeed) - initTime;
finalTime2 = (distance2/toolSpeed) - initTime;
trajTimes1 = initTime:timeStep:finalTime1;
trajTimes2 = initTime:timeStep:finalTime2;

%�ж����۵��˶�ʱ�䣬ѡȡʱ��ϳ���һ����Ϊϵͳ�˶�ʱ��
if length(trajTimes1)>length(trajTimes2)
    trajTimes=trajTimes1;
else
    trajTimes=trajTimes2;
end

timeInterval= [trajTimes(1); trajTimes(end)];
%%  ���۹ؽڿռ�켣������
ik1 = inverseKinematics('RigidBodyTree',robot_arm1);
ik1.SolverParameters.AllowRandomRestart = false;
weights = [0 0 0 1 1 1];

initialGuess = wrapToPi(jointInit);
[jointFinal1,info1] = ik1(endEffector,taskFinal,weights,initialGuess);
jointFinal1 = wrapToPi(jointFinal1);
%���ɹ켣��
ctrlpoints1 = [jointInit,jointFinal1];
jointConfigArray1 = cubicpolytraj(ctrlpoints1,timeInterval,trajTimes);%���ζ���ʽ�켣

%%  ���۹ؽڿռ�켣������
ik2 = inverseKinematics('RigidBodyTree',robot_arm2);
ik2.SolverParameters.AllowRandomRestart = false;

[jointFinal2,info2] = ik2(endEffector,taskFinal_,weights,initialGuess);
jointFinal2 = wrapToPi(jointFinal2);
%���ɹ켣��
ctrlpoints2 = [jointInit,jointFinal2];
jointConfigArray2 = cubicpolytraj(ctrlpoints2,timeInterval,trajTimes);%���ζ���ʽ�켣

%%  ��ײģ�͵���

collisionArrayFromMesh_1=cell(robot_arm1.NumBodies,2);
collisionArrayFromMesh_2=cell(robot_arm2.NumBodies,2);

robotarm_Collision_1 = robot_arm1;
robotarm_Collision_2 = robot_arm2;

% ȡ��������Ӧ�ؽڵ� STL �ļ��е�ģ�����ݡ�����е��ĩ�˳���
robotBodies_1 = [{robotarm_Collision_1.Base} robotarm_Collision_1.Bodies];%%�����������ݱ�ʾ����е�ۻ�����������
for i = 1:numel(robotBodies_1)-1  %������е��ĩ�˳���
    if ~isempty(robotBodies_1{i}.Visuals)
        % Assumes the first Visuals element is the correct one.
        visualDetails = robotBodies_1{i}.Visuals{1};
        
        % Extract the part of the visual that actually specifies the STL name
        visualParts = strsplit(visualDetails, ':');
        stlFileName = visualParts{2};
        
        % Read the STL file
        stlData = stlread(stlFileName);
        
        % Create a collisionMesh object from the vertices
        collisionArrayFromMesh_1{i,1} = collisionMesh(stlData.Points);
        
        % Transform is always identity
        collisionArrayFromMesh_1{i,2} = eye(4);
    end
end
   

% ȡ��������Ӧ�ؽڵ� STL �ļ��е�ģ�����ݡ�����е��ĩ�˳���
robotBodies_2 = [{robotarm_Collision_2.Base} robotarm_Collision_2.Bodies];%%�����������ݱ�ʾ����е�ۻ�����������
for i = 1:numel(robotBodies_2)-1  %������е��ĩ�˳���
    if ~isempty(robotBodies_2{i}.Visuals)
        % Assumes the first Visuals element is the correct one.
        visualDetails = robotBodies_2{i}.Visuals{1};
        
        % Extract the part of the visual that actually specifies the STL name
        visualParts = strsplit(visualDetails, ':');
        stlFileName = visualParts{2};
        
        % Read the STL file
        stlData = stlread(stlFileName);
        
        % Create a collisionMesh object from the vertices
        collisionArrayFromMesh_2{i,1} = collisionMesh(stlData.Points);
        
        % Transform is always identity
        collisionArrayFromMesh_2{i,2} = eye(4);
    end
end

%%  ��ʾ
figure
show(robot_arm1,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
show(robot_arm2,currentRobotJConfig,'PreservePlot',false,'Frames','off');
axis([-0.5 0.8 -0.5 0.5 -0.1 0.5]);
camva('auto');%��������ӽ�

%����ӰƬ֡����
F(length(trajTimes2)) = struct('cdata',[],'colormap',[]);

for i=1:length(trajTimes)
     % Current time 
    tNow= trajTimes(i);
    
    % robot_arm1�ؽڿռ�켣
    configNow1 = jointConfigArray1(:,i);
    poseNow1 = getTransform(robot_arm1,configNow1,endEffector);

    % robot_arm2�ؽڿռ�켣
    configNow2 = jointConfigArray2(:,i);
    poseNow2 = getTransform(robot_arm2,configNow2,endEffector);
     
     %ÿһʱ��˫��е�۹ؽڼ���ײ���
    [isCollision, collisionPairIdx] = dualmanipsCheckCollisions(robot_arm1,robot_arm2,...
        collisionArrayFromMesh_1, collisionArrayFromMesh_2,configNow1, configNow2, true);
    disp(isCollision)
    disp(collisionPairIdx)
    
    show(robot_arm1,configNow1,'PreservePlot',false,'Frames','off');
    plot3(poseNow1(1,4),poseNow1(2,4),poseNow1(3,4),'b.','MarkerSize',10)
    
    if isCollision==1
        % ������ײ�ؽڱ��
        highlightCollisionBodies_Arms(robot_arm1,collisionPairIdx(:,1),gca)
    end
    
    show(robot_arm2,configNow2,'PreservePlot',false,'Frames','off');
    plot3(poseNow2(1,4),poseNow2(2,4),poseNow2(3,4),'r.','MarkerSize',10)
    
    if isCollision==1
        % ������ײ�ؽڱ��
        highlightCollisionBodies_Arms(robot_arm2,collisionPairIdx(:,2),gca)
    end
    
    drawnow;
   
    %����ͼƬ֡
    F(i) = getframe(gcf);
end

%% ���Ŷ���
fig = figure;
movie(fig,F,5,10)

