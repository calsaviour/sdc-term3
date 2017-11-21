function [ O ] = GenerateStaircaseObstacles( num_obs, margins )
%GenerateStaircaseObstacles Summary of this function goes here
%   Detailed explanation goes here
    
    % Limits on the system
    x_limit = getMargin(margins, 'x_limit') - 1;
    t_limit = getMargin(margins, 't_limit') - 0.5;
    
    % From the limits, generate a staircase configuration of obstacles
    O = zeros(num_obs, 4);
    for i=1:num_obs
        center_x = i * 0.9;
        center_t = i * 0.9;
        O(i, :) = [center_x-0.25 center_x+0.25 center_t-0.25 center_t+0.25];
    end

end

