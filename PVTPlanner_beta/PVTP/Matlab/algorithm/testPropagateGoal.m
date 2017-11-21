function [ S ] = testPropagateGoal( p0, V_i, V_f, O, margins )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


    % Get obstacle corner points
    [UL LR] = segregatePoints(O);
    
    % Add point type designation to each:
    % UL = 1
    % LR = 0
    
    s_UL = size(UL);
    p_type = ones(s_UL(1, 1), 1);
    UL = cat(2, UL, p_type);
    
    s_LR = size(LR);
    p_type = zeros(s_UL(1, 1), 1);
    LR = cat(2, LR, p_type);
    
    % Combine into one set
    P = cat(1, UL, LR);
    
    % Add origin (a lower-right point by convention)
    P = cat(1, P, [0 0 0]);
    s_P = size(P);
    
    % Sort by time coordinate
    P_t = sortrows(P, [2 1]);

    S = struct('UB', [], 'LB', [], 'B', []);
    S = PropagateGoal( p0, V_i, V_f, S, O, P_t, margins);
    

end

