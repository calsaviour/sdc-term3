function [ times, final_intervals, intermediate_intervals, G, Goal, O ] = Benchmark2( num_obs, inc, O, margins )
%Benchmark2 Summary of this function goes here
%   Detailed explanation goes here

    if isempty(O)
        O = GenerateRandomObstacles(num_obs, margins);
    end
    
    % Parameters
    V_i = [0 0];
    V_g = [5 10];
    
    % Store times in matrix: [#obs time]
    times = [];
    
    % Store final intervals in matrix: [#obs intervals]
    final_intervals = [];
    
    % Store intermediate intervals in matrix: [#obs intervals]
    intermediate_intervals = [];
    
    for i=inc:inc:num_obs
        
        % Display progress
        display(['Testing ', num2str(i), ' obstacles...']);
        
        % Get start time
        tic;
        
        % Generate reachable sets
        [G Goal inter_intervals] = Forward([0 0], V_i, V_g, O(1:i, :), margins);
        
        % Get optimal trajectory
        T = GetOptimalTrajectory(G, Goal, O(1:i, :), margins);
        
        % Get end time
        time = toc;
        
        % Update times
        times = cat(1, times, [i time]);
        
        % Update interval count
        interval_count = CountVelocityIntervals(G);
        final_intervals = cat(1, final_intervals, [i interval_count]);
        
        % Update intermediate interval count
        intermediate_intervals = cat(1, intermediate_intervals, [i inter_intervals]);
        
    end
    
    % Normalize the times
    base_time = times(1, 2);
    times(:, 2) = times(:, 2) / base_time;
    
    % Find a polynomial fit... degree 3 works nicely
    p = polyfit(times(:, 1), times(:, 2), 3);
    
    % Get fit values
    f = polyval(p, times(:, 1));
    
    % Plot values
    %plot(times(:, 1), times(:, 2), 'o', times(:, 1), f, '-');
    
end

