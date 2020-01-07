clear
clc
%% 版本说明：

%去掉关节运动模型，直接使用三次多项式轨迹规划结果——2019.12.25

%% robot_arm URDF
addpath(genpath('urdf'),genpath('Utilities'),genpath('simulink'))
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

