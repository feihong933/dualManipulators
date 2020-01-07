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
taskInit1 = getTransform(robot_arm1,jointInit,endEffector);
taskInit2 = getTransform(robot_arm2,jointInit,endEffector);
% taskFinal = trvec2tform([0.3,0.1,0.25])*axang2tform([1 0 0 -pi/2])
taskFinal = trvec2tform([0.2,-0.1,0.35]);
taskFinal_ = trvec2tform([-0.07,-0.1,0.35]);
display(taskFinal)
display(taskFinal_)
%compute tool traveling distance
distance1 = norm(tform2trvec(taskInit1)-tform2trvec(taskFinal));
distance2 = norm(tform2trvec(taskInit2)-tform2trvec(taskFinal));
%define trajectory times based on traveling distance and desired tool speed
initTime = 0;
finalTime1 = (distance1/toolSpeed) - initTime;
finalTime2 = (distance2/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime1;
timeInterval = [trajTimes(1); trajTimes(end)];

%%主臂关节空间轨迹
ik1 = inverseKinematics('RigidBodyTree',robot_arm1);
ik1.SolverParameters.AllowRandomRestart = false;
weights = [0 0 0 1 1 1];

initialGuess = wrapToPi(jointInit);
[jointFinal1,info1] = ik1(endEffector,taskFinal,weights,initialGuess);
jointFinal1 = wrapToPi(jointFinal1);
%生成轨迹点
ctrlpoints1 = [jointInit',jointFinal1'];
jointConfigArray1 = cubicpolytraj(ctrlpoints1,timeInterval,trajTimes);%三次多项式轨迹
jointWaypoints1 = bsplinepolytraj(jointConfigArray1,timeInterval,trajTimes);%B样条轨迹

%创建关节空间运动模型
jsMotionModel1 = jointSpaceMotionModel('RigidBodyTree',robot_arm2,'MotionType','PDControl');
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

%Use ode15s to simulate the robot motion. Again, an example helper function is used as the function handle input to the ODE solver in order to update 
%the reference inputs at each instant in time. The joint-space states are output in stateJoint.

[tJoint1,stateJoint1] = ode15s(@(t,state) TimeBasedJointInputs(jsMotionModel1,timeInterval,jointConfigArray1,t,state),timeInterval,[q0; qd0]);


%%副臂关节空间轨迹
ik2 = inverseKinematics('RigidBodyTree',robot_arm2);
ik2.SolverParameters.AllowRandomRestart = false;


[jointFinal2,info2] = ik2(endEffector,taskFinal_,weights,initialGuess);
jointFinal2 = wrapToPi(jointFinal2);
%生成轨迹点
ctrlpoints2 = [jointInit',jointFinal2'];
jointConfigArray2 = cubicpolytraj(ctrlpoints2,timeInterval,trajTimes);%三次多项式轨迹
jointWaypoints2 = bsplinepolytraj(jointConfigArray2,timeInterval,trajTimes);%B样条轨迹

%创建关节空间运动模型
jsMotionModel2 = jointSpaceMotionModel('RigidBodyTree',robot_arm2,'MotionType','PDControl');
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

%Use ode15s to simulate the robot motion. Again, an example helper function is used as the function handle input to the ODE solver in order to update 
%the reference inputs at each instant in time. The joint-space states are output in stateJoint.

[tJoint2,stateJoint2] = ode15s(@(t,state) TimeBasedJointInputs(jsMotionModel2,timeInterval,jointConfigArray2,t,state),timeInterval,[q0; qd0]);


figure
show(robot_arm1,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
show(robot_arm2,currentRobotJConfig,'PreservePlot',false,'Frames','off');
axis([-0.5 0.8 -0.5 0.5 -0.1 0.5]);
camva('auto');%设置相机视角


for i=1:length(trajTimes)
     % Current time 
    tNow= trajTimes(i);
    
    % robot_arm1关节空间轨迹
    % Interpolate simulated joint positions to get configuration at current time
    configNow1 = interp1(tJoint1,stateJoint1(:,1:numJoints),tNow);
    poseNow1 = getTransform(robot_arm1,configNow1,endEffector);
    show(robot_arm1,configNow1,'PreservePlot',false,'Frames','off');
    plot3(poseNow1(1,4),poseNow1(2,4),poseNow1(3,4),'b.','MarkerSize',10)
    
    % robot_arm2关节空间轨迹
    % Interpolate simulated joint positions to get configuration at current time
    configNow2 = interp1(tJoint2,stateJoint2(:,1:numJoints),tNow);
    poseNow2 = getTransform(robot_arm2,configNow2,endEffector);
    show(robot_arm2,configNow2,'PreservePlot',false,'Frames','off');
    plot3(poseNow2(1,4),poseNow2(2,4),poseNow2(3,4),'r.','MarkerSize',10)
    drawnow;
end





