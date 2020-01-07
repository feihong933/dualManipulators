clear
clc
%% �汾˵����

%ȥ���ؽ��˶�ģ�ͣ�ֱ��ʹ�����ζ���ʽ�켣�滮�������2019.12.25

%% robot_arm URDF
addpath(genpath('urdf'),genpath('Utilities'),genpath('simulink'))
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

