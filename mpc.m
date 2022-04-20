
% Playground for the project
%
% Position based variable names:
% WF = world frame
% RF = robot frame
%
% Non-holonomic robot
% Control - (v = linear velocity,theta = angular position)'
% Position uses flat surface assumption (x,y,theta)

clc;
clear;
% Create objects
display = GUI();
robot = RobotDriver(0.05,[1;1;pi/4],display); % Initialize robot with tick speed and pose - (x,y,theta) in world frame.

display.drawPoint(robot.position);
global goalPositionWF;
goalPositionWF = [30;100]; % Goal position in world frame (x,y)
display.drawPoint(goalPositionWF);

% Perform robot ops
robot.nodeObservationList('Node 1') = goalPositionWF;
% robot.moveRobotForTicks(1,0,10);
% robot.go2Node('Node 1',0.1,[0.9;0.9]);
Np = 5;
Nc = 5;
sstar = [0;0];
s = robot.getNodeLocation('Node 1');
L = [[-1,s(2)];[0,-s(1)]];
sm = s; % Init
x0 = ones(1,Np*2);
model_twists = zeros(Np,2);

while sqrt((0 - s(1))^2 + (0 - s(2))^2) > 0
    timeHorizon = robot.timeHorizon;
    f = @(x)JGo2Node(x,sstar,s,sm,L,Np,timeHorizon,display);
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-20*ones(1,Np),-5*ones(1,Np)];
    ub = [20*ones(1,Np),5*ones(1,Np)];
    nonlcon = [];
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
    [x,fval,exitflag,output] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    twists = x;
    x0 = twists; % TODO remove redundant variables
    
    for i = 1:Nc
        robot.moveRobotForTicks(twists(i),twists(Np+i),1);
        s = robot.getNodeLocation('Node 1');
        L = [[-1,sm(2)];[0,-sm(1)]];
        sm = sm + L*timeHorizon*[twists(i);twists(Np+i)];
    end
    model_twists = [model_twists;reshape(twists,[Np,2])];
    display.plotModelTwist(reshape(twists,[Np,2]));
end

display.drawDirectionalPoint(robot.position);
