function J = JGo2Node(x,sstar,s,sm,L,Np,timeHorizon,display)
    e = s - sm;
    sd = sstar - e;
    
    x = reshape(x,[Np,2]); % reshapes as [[x1,x6];[x2,x7];....]
    Jtemp = 0;
    for temp = 1:Np
        sm = sm + L*timeHorizon*x(temp,:)';
        Jtemp = Jtemp + (sd - sm)'*(sd - sm);
    end
%     display.plotCost(Jtemp);
    J = Jtemp;
end