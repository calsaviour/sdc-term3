function [ O ] = GenerateRandomObstacles( num_obs, margins )
%GenerateRandomObstacles Summary of this function goes here
%   Detailed explanation goes here

    % Get a bunch of random numbers
    rand_nums = rand(1, num_obs*2);
    
    % Limits on the system
    x_limit = getMargin(margins, 'x_limit') - 2;
    t_limit = getMargin(margins, 't_limit') - 1;
    
    % From the limits, generate a bunch of random objects 2 x 1
    O = zeros(num_obs, 4);
    for i=1:num_obs
        center_x = rand_nums(i) * (x_limit - 1) + 1;
        center_t = rand_nums(i+num_obs) * (t_limit - 0.5) + 0.5;
        O(i, :) = [center_x-0.75 center_x+0.75 center_t-0.25 center_t+0.25];
    end

end

