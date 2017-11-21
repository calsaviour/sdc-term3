function [ B ] = testHomotopic( T, O, margins )
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
    p0 = [0 0];
    P = cat(1, P, [p0 0]);
    s_P = size(P);
    
    % Sort by time coordinate
    P_t = sortrows(P, [2 1]);
    
    trials = 1;
    tic;
    for i=1:trials
        B = Homotopic( T, O, P_t, margins );
    end
    time = (toc / 10) * 1000000;
    display([num2str(trials), ' trials, time elapsed: ', num2str(time), ' microseconds']);
    
end

