clear
clc

% robot_arm URDF
addpath(genpath('urdf'))
robot_arm1 = importrobot('manipulator1.urdf');
robot_arm2 = importrobot('manipulator2.urdf');

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

homeRobotConfig1= homeConfiguration(robot_arm1);
homeRobotConfig2 = homeConfiguration(robot_arm2);
randomRobotConfig=robot_arm1.randomConfiguration;
targetRobotConfig1=[pi/2,pi/2,pi/2,pi/2,pi/2];
targetRobotConfig2=[pi/2,pi/2,0,pi/2,pi/2];
% figure
% show(robot_arm1,targetRobotConfig1 ,'Frames','off');

% showdetails(robot_arm);
% getTransform(robot_arm1,targetRobotConfig,'link5')


tform=getTransform(robot_arm1,targetRobotConfig1,'end_effector_link');

%逆解
% ik=robotics.InverseKinematics('RigidBodyTree',robot_arm1);
% weights= [ 1 1 1 0.05 0.05 0.05];
% initialguess=homeRobotConfig1;
% % ik.SolverParameters.AllowRandomRestarts = true;
% % ik.SolverParameters.MaxIterations = 1500;
% [config,info]=ik('end_effector_link',tform,weights,initialguess);


currentRobotJConfig = homeConfiguration(robot_arm1);
numJoints = numel(currentRobotJConfig);
endEffector = "end_effector_link";

timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

%Set the initial and final end-effector pose.
jointInit = currentRobotJConfig;
taskInit1 = getTransform(robot_arm1,jointInit,endEffector)
taskInit2 = getTransform(robot_arm2,jointInit,endEffector)
% taskFinal = trvec2tform([0.3,0.1,0.25])*axang2tform([1 0 0 -pi/2])
taskFinal = trvec2tform([0.07,0.1,0.35])

%compute tool traveling distance
distance1 = norm(tform2trvec(taskInit1)-tform2trvec(taskFinal));
distance2 = norm(tform2trvec(taskInit2)-tform2trvec(taskFinal));
%define trajectory times based on traveling distance and desired tool speed
initTime = 0;
finalTime1 = (distance1/toolSpeed) - initTime;
finalTime2 = (distance2/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime1;
timeInterval = [trajTimes(1); trajTimes(end)];

%任务空间轨迹
[taskWaypoints,taskVelocities] = transformtraj(taskInit1,taskFinal,timeInterval,trajTimes); 

%创建任务空间机械臂模型
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot_arm1,'EndEffectorName','end_effector_link');
 tsMotionModel.Kp(1:3,1:3) = 0;
 tsMotionModel.Kd(1:3,1:3) = 0;

%Define the initial states (joint positions and velocities).
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

[tTask,stateTask] = ode15s(@(t,state) TimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit1,taskFinal,t,state),timeInterval,[q0; qd0]);


%%关节空间轨迹
ik = inverseKinematics('RigidBodyTree',robot_arm2);
ik.SolverParameters.AllowRandomRestart = false;
weights = [0 0 0 1 1 1];

initialGuess = wrapToPi(jointInit);
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);
jointFinal = wrapToPi(jointFinal);
%生成轨迹点
ctrlpoints = [jointInit',jointFinal'];
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);%三次多项式轨迹
jointWaypoints = bsplinepolytraj(jointConfigArray,timeInterval,trajTimes);%B样条轨迹

%创建关节空间运动模型
jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot_arm2,'MotionType','PDControl');
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

%Use ode15s to simulate the robot motion. Again, an example helper function is used as the function handle input to the ODE solver in order to update 
%the reference inputs at each instant in time. The joint-space states are output in stateJoint.

[tJoint,stateJoint] = ode15s(@(t,state) TimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; qd0]);


figure
show(robot_arm1,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
show(robot_arm2,currentRobotJConfig,'PreservePlot',false,'Frames','off');
axis([-0.5 0.8 -0.5 0.5 -0.1 0.5]);



for i=1:length(trajTimes)
     % Current time 
    tNow= trajTimes(i);
    
    %Visualize the task-space trajectory.
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(robot_arm1,configNow,endEffector);
    show(robot_arm1,configNow,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',10)
    
    %Visualize the joint-space trajectory
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tJoint,stateJoint(:,1:numJoints),tNow);
    poseNow = getTransform(robot_arm2,configNow,endEffector);
    show(robot_arm2,configNow,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',10)
    drawnow;
end

% figure
% show(robot_arm1,config ,'Frames','off');
%  hold on
% show(robot_arm2,targetRobotConfig1,'Frames','off');
% 
% % view([1024 960]);%figure大小设置
% axis([-0.5 0.8 -0.5 0.5 -0.1 0.5]);%坐标范围
% camva('auto');%设置相机视角




