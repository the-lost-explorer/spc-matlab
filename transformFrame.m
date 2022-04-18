% coordinate transformation functions

function tf = transformFrame(node,robotPose)
    RthetaRobot = Rtheta(robotPose);
    newLocation = RthetaRobot*[[node(1)-robotPose(1);node(2)-robotPose(2)];1];
    tf = [newLocation(1);newLocation(2)];
end

function rtheta = Rtheta(pose)
    rtheta = [[cos(-pose(3)), -sin(-pose(3)),  0 ];
              [sin(-pose(3)),  cos(-pose(3)),  0 ];
              [0,                0,             1    ]];
end