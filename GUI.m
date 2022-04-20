% Class providing a GUI
classdef GUI < handle
    properties (SetAccess = private, GetAccess = public)
        fig;
        stats;
        twists;
        cost;
    end
    
    methods
        function self = GUI()
            self.twists = zeros(1,2);
            self.fig = figure('Name','Robot Movement','NumberTitle','off');
            self.cost = 0;
        end
        
        function drawPoint(self,point)
            figure(self.fig);
            axis equal;
            hold on;
            subplot(6,4,[1,2,3,4,5,6,7,8,9,10,11,12]);
            plot(point(1),point(2),'*');
            hold off;
        end
        
        function drawDirectionalPoint(self,dpoint)
            figure(self.fig);
            axis equal;
            hold on;
            subplot(6,4,[1,2,3,4,5,6,7,8,9,10,11,12]);
            arrow([dpoint(1),dpoint(2)],[dpoint(1)+0.05*cos(dpoint(3)),dpoint(2)+0.05*sin(dpoint(3))]);
            hold off;
        end
        
        function drawTrajectory(self,trajectory)
            figure(self.fig);
            axis equal;
            hold on;
            subplot(6,4,[1,2,3,4,5,6,7,8,9,10,11,12]);
            plot(trajectory(:,1),trajectory(:,2));
            hold off;
        end
        
        function plotTwist(self,twist)
            self.twists = [self.twists;twist'];
            figure(self.fig);
            hold on;
            subplot(6,4,[13,14]);
            plot(self.twists(:,1));
            ylabel('vx');
            hold off;
            
            hold on;
            subplot(6,4,[17,18]);
            plot(self.twists(:,2));
            ylabel('wz');
            hold off;
        end
        
        function plotModelTwist(self,model_twists)
            figure(self.fig);
            subplot(6,4,[15,16]);
            cla();
            plot(model_twists(:,1));
            ylabel('vmx');
            
            subplot(6,4,[19,20]);
            cla();
            plot(model_twists(:,2));
            ylabel('wmz');
            
        end
        
        function plotCost(self,cost)
            self.cost = [self.cost;cost];
            figure(self.fig);
            hold on;
            subplot(6,4,[21,22,23,24]);
            plot(self.cost);
            ylabel('J');
            hold off;
        end
    end
end