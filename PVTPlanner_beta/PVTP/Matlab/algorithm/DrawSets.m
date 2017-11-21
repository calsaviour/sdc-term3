function [ ] = DrawSets( G, Goal, O, margins )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    drawObstacles(O, margins);
    DrawReachableSets(G, margins);
    DrawGoalRanges(Goal);
    
end

