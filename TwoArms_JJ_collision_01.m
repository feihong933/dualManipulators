clear
clc
%% 版本说明：

%去掉关节运动模型，直接使用三次多项式轨迹规划结果——2019.12.25

%改进碰撞检测模型，用圆柱简化

%% robot_arm URDF
addpath(genpath('urdf'),genpath('Utilities'))
robot_arm1 = importrobot('manipulator1.urdf');
robot_arm2 = importrobot('manipulator2.urdf');

% 设置数据格式
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

%% 起始点等参数设置
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

%判断两臂的运动时间，选取时间较长的一方作为系统运动时间
if length(trajTimes1)>length(trajTimes2)
    trajTimes=trajTimes1;
else
    trajTimes=trajTimes2;
end

timeInterval= [trajTimes(1); trajTimes(end)];
%%  主臂关节空间轨迹点生成
ik1 = inverseKinematics('RigidBodyTree',robot_arm1);
ik1.SolverParameters.AllowRandomRestart = false;
weights = [0 0 0 1 1 1];

initialGuess = wrapToPi(jointInit);
[jointFinal1,info1] = ik1(endEffector,taskFinal,weights,initialGuess);
jointFinal1 = wrapToPi(jointFinal1);
%生成轨迹点
ctrlpoints1 = [jointInit,jointFinal1];
jointConfigArray1 = cubicpolytraj(ctrlpoints1,timeInterval,trajTimes);%三次多项式轨迹

%%  副臂关节空间轨迹点生成
ik2 = inverseKinematics('RigidBodyTree',robot_arm2);
ik2.SolverParameters.AllowRandomRestart = false;

[jointFinal2,info2] = ik2(endEffector,taskFinal_,weights,initialGuess);
jointFinal2 = wrapToPi(jointFinal2);
%生成轨迹点
ctrlpoints2 = [jointInit,jointFinal2];
jointConfigArray2 = cubicpolytraj(ctrlpoints2,timeInterval,trajTimes);%三次多项式轨迹

%%  碰撞模型导入

collision1=importrobot('collision1.urdf');
collision2=importrobot('collision2.urdf');

collisionArrayFromMesh_1=cell(robot_arm1.NumBodies,2);
collisionArrayFromMesh_2=cell(robot_arm2.NumBodies,2);

robotarm_Collision_1 = collision1;
robotarm_Collision_2 = collision1;


% 主臂碰撞检测模型——机械臂末端除外
robotBodies_1 = [{robotarm_Collision_1.Base} robotarm_Collision_1.Bodies];%%大括号中内容表示将机械臂基座包括在内

collisionArrayFromMesh_1{1,1} = collisionCylinder(0.07,0.08);
collisionArrayFromMesh_1{1,2}=trvec2tform([0,0,0.045]);

collisionArrayFromMesh_1{2,1} = collisionCylinder(0.04,0.05);
collisionArrayFromMesh_1{2,2}=trvec2tform([0,0,0.03]);

collisionArrayFromMesh_1{3,1} = collisionCylinder(0.028,0.1);
collisionArrayFromMesh_1{3,2}=trvec2tform([0.05,0,0])*axang2tform([0 1 0 pi/2]);

collisionArrayFromMesh_1{4,1} = collisionCylinder(0.028,0.09);
collisionArrayFromMesh_1{4,2}=trvec2tform([0.05,0,0.0])*axang2tform([0 1 0 pi/2]);

collisionArrayFromMesh_1{5,1} = collisionCylinder(0.025,0.07);
collisionArrayFromMesh_1{5,2}=trvec2tform([0.05,0,0])*axang2tform([0 1 0 pi/2]);

collisionArrayFromMesh_1{6,1} = collisionBox(0.06,0.02,0.09);
collisionArrayFromMesh_1{6,2}=trvec2tform([0,0,0.075]);



% 副臂碰撞模型——机械臂末端除外
robotBodies_2 = [{robotarm_Collision_2.Base} robotarm_Collision_2.Bodies];%%大括号中内容表示将机械臂基座包括在内

collisionArrayFromMesh_2{1,1} = collisionCylinder(0.07,0.08);
collisionArrayFromMesh_2{1,2}=trvec2tform([0.3,0,0.045])*axang2tform([0 0 1 2*pi]);

collisionArrayFromMesh_2{2,1} = collisionCylinder(0.04,0.05);
collisionArrayFromMesh_2{2,2}=trvec2tform([0,0,0.03]);

collisionArrayFromMesh_2{3,1} = collisionCylinder(0.028,0.1);
collisionArrayFromMesh_2{3,2}=trvec2tform([0.05,0,0])*axang2tform([0 1 0 pi/2]);

collisionArrayFromMesh_2{4,1} = collisionCylinder(0.028,0.09);
collisionArrayFromMesh_2{4,2}=trvec2tform([0.05,0,0.0])*axang2tform([0 1 0 pi/2]);

collisionArrayFromMesh_2{5,1} = collisionCylinder(0.025,0.07);
collisionArrayFromMesh_2{5,2}=trvec2tform([0.05,0,0])*axang2tform([0 1 0 pi/2]);

collisionArrayFromMesh_2{6,1} = collisionBox(0.06,0.02,0.09);
collisionArrayFromMesh_2{6,2}=trvec2tform([0,0,0.075]);


%%  显示
figure
show(robot_arm1,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
% showCollisionTree(robot_arm1,collisionArrayFromMesh_1,randomRobotConfig);

show(robot_arm2,currentRobotJConfig,'PreservePlot',false,'Frames','off');
% showCollisionTree(robot_arm2,collisionArrayFromMesh_2,randomRobotConfig);
axis([-0.5 0.8 -0.5 0.5 -0.1 0.5]);
camva('auto');%设置相机视角

%定义影片帧矩阵
F(length(trajTimes2)) = struct('cdata',[],'colormap',[]);

for i=1:length(trajTimes)
     % Current time 
    tNow= trajTimes(i);
    
    % robot_arm1关节空间轨迹
    configNow1 = jointConfigArray1(:,i);
    poseNow1 = getTransform(robot_arm1,configNow1,endEffector);

    % robot_arm2关节空间轨迹
    configNow2 = jointConfigArray2(:,i);
    poseNow2 = getTransform(robot_arm2,configNow2,endEffector);
     
     %每一时刻双机械臂关节间碰撞检测
    [isCollision, collisionPairIdx] = dualmanipsCheckCollisions(robot_arm1,robot_arm2,...
        collisionArrayFromMesh_1, collisionArrayFromMesh_2,configNow1, configNow2, true);
    disp(isCollision)
    disp(collisionPairIdx)
    
    show(robot_arm1,configNow1,'PreservePlot',false,'Frames','off');
%     showCollisionTree(robot_arm1,collisionArrayFromMesh_1,configNow1);
    
    plot3(poseNow1(1,4),poseNow1(2,4),poseNow1(3,4),'b.','MarkerSize',10)
    
    if isCollision==1
        % 主臂碰撞关节标红
        highlightCollisionBodies_Arms(robot_arm1,collisionPairIdx(:,1),gca)
    end
    
    show(robot_arm2,configNow2,'PreservePlot',false,'Frames','off');
%     showCollisionTree(robot_arm2,collisionArrayFromMesh_2,configNow2);
   
    plot3(poseNow2(1,4),poseNow2(2,4),poseNow2(3,4),'r.','MarkerSize',10)
    
    if isCollision==1
        % 副臂碰撞关节标红
        highlightCollisionBodies_Arms(robot_arm2,collisionPairIdx(:,2),gca)
    end
    
%     figure
%     showCollisionTree(robot_arm1,collisionArrayFromMesh_1,configNow1);
%     hold on
%     showCollisionTree(robot_arm2,collisionArrayFromMesh_2,configNow2);
%     
    drawnow;
   
    %保存图片帧
    F(i) = getframe(gcf);
end

%% 播放动画
fig = figure;
movie(fig,F,5,10)

