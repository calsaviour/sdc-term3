function [ ] = DrawGoalRanges( Goal )
%DrawGoalRanges Summary of this function goes here
%   Detailed explanation goes here

    s_Goal = size(Goal);
    for i=1:s_Goal(1, 2)
        
        UB = Goal(i).UB;
        LB = Goal(i).LB;
        
        drawTrajectory(UB);
        drawTrajectory(LB);
        
    end

end

