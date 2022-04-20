% Robot movement based on p,v,w,t
classdef RobotDriver < handle
    properties
        timeHorizon = 0.05;
        nodeObservationList; % Contains a map of directional points in world frame being observed by the robot
        gui;
    end
    properties (SetAccess = private, GetAccess = public)
        trajectory = zeros(1,3); % Set of poses robot assumes while travelling through the world in world frame
        position = [0,0,0]; % This is robot position in world frame
    end
    methods
        function self = RobotDriver(timeHorizon,position,gui)
            if nargin > 0
                self.timeHorizon = timeHorizon;
                self.position = position;
                self.trajectory = position';
                self.gui = gui;
            end
            self.nodeObservationList = containers.Map;
        end

        function moveRobotForTicks(self,xVelocity,zAngularVelocity,ticks)
            % This function performs the forward kinematics and returns a
            % trajectory of motion.
            for temp=1:ticks
                L = [[cos(self.position(3)), 0];
                     [sin(self.position(3)), 0];
                     [0                    , 1]];
                DPos = L*[xVelocity*self.timeHorizon;zAngularVelocity*self.timeHorizon];
                self.position = self.position + DPos;
                self.trajectory = [self.trajectory;self.position'];
                self.gui.drawTrajectory(self.trajectory);
                self.gui.plotTwist([xVelocity;zAngularVelocity]);
            end
        end
        function node = getNodeLocation(self,nodeKey) % Returns node location in robot frame
            node = transformFrame(self.nodeObservationList(nodeKey),self.position);
        end
        
        function go2Node(self,nodeKey,goalThreshold,twistGain)
            node = self.getNodeLocation(nodeKey);
            errorTracker = [node(1);node(2)]';
            while sqrt((0 - node(1))^2 + (0 - node(2))^2) > goalThreshold
                node = self.getNodeLocation(nodeKey);
                L = [[-1,node(2)];[0,-node(1)]];
                e = [node(1);node(2)];
                twist = -twistGain.*(pinv(L)*e);
                self.moveRobotForTicks(twist(1),twist(2),1);
                errorTracker = [errorTracker;e'];
            end
           figure('Name','Error Tracker','NumberTitle','off'); 
           hold on;
           plot(errorTracker(:,1));
           plot(errorTracker(:,2));
           legend('x-error','y-error');
           hold off;
        end
        
        function go2Pose(self,pose,goalThreshold,twistGain)
            errorTracker = [pose(1);pose(2)]';
            dist = sqrt((pose(1)*cos(pose(3)) - pose(1))^2 + (pose(2)*sin(pose(3)) - pose(2))^2);
            while sqrt((0 - pose(1))^2 + (0 - pose(2))^2) > goalThreshold
                pose = transformFrame(pose,self.position);
                L = [[-1,pose(2)];[0,-pose(1)];[-1, pose(2)*sin(pose(3))];[0, -pose(1)*cos(pose(3)) ]];
                e = [pose(1);pose(2);pose(1)*cos(pose(3))-dist;pose(2)*sin(pose(3))];
                twist = -twistGain.*(pinv(L)*e);
                self.moveRobotForTicks(twist(1),twist(2),1);
                errorTracker = [errorTracker;e'];
            end
           figure('Name','Error Tracker','NumberTitle','off'); 
           hold on;
           plot(errorTracker(:,1));
           plot(errorTracker(:,2));
           legend('x-error','y-error');
           hold off;
        end
    end
end