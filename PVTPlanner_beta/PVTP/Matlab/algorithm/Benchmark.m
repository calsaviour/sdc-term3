function [ times, final_intervals, intermediate_intervals, G, Goal ] = Benchmark( num_obs, inc, margins )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here


    % Generate some worst case obstacles
    WORST_CASE = [2 3 2 3];
    for i=1:num_obs-1
        WORST_CASE = cat(1, WORST_CASE, [3 3 3 3]+WORST_CASE(end, :));
    end

    % Set limits past all obstacles
    last_x_corner = getObstacleCorner(WORST_CASE(end, :), 'lower-right');
    last_t_corner = getObstacleCorner(WORST_CASE(end, :), 'upper-left');
    x_limit = getStateCoord(last_x_corner, 'x') + 3;
    t_limit = getStateCoord(last_t_corner, 't') + 3;
    
    margins(1, 10) = x_limit;
    margins(1, 11) = t_limit;
    
    % Double check
    if x_limit ~= getMargin(margins, 'x_limit')...
            || t_limit ~= getMargin(margins, 't_limit')
        display('Limits set incorrectly.');
    end    
    
    % Parameters
    V_i = [0 0];
    V_g = [5 10];
    
    % Get base case
    [G Goal] = Forward([0 0], V_i, V_g, WORST_CASE(1, :), margins);
    
    % Store times in matrix: [#obs time]
    times = [];
    
    % Store final intervals in matrix: [#obs intervals]
    final_intervals = [];
    
    % Store intermediate intervals in matrix: [#obs intervals]
    intermediate_intervals = [];
    
    % Run the algorithm on sets of the obstacles
    for i=inc:inc:num_obs
        
        % Display progress
        display(['Testing ', num2str(i), ' obstacles...']);
        
        % Get start time
        tic;
        
        % Generate reachable sets
        [G Goal inter_intervals] = Forward([0 0], V_i, V_g, WORST_CASE(1:i, :), margins);
        
        % Get optimal trajectory
        T = GetOptimalTrajectory(G, Goal, WORST_CASE(1:i, :), margins);
        
        % Get end time
        time = toc;
        
        % Update times
        times = cat(1, times, [i time]);
        
        % Update final interval count
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
    plot(times(:, 1), times(:, 2), 'o', times(:, 1), f, '-');
    
    %drawObstacles(WORST_CASE(1:num_obs, :), margins);
    %DrawReachableSets(G, margins);
    %DrawGoalRanges(Goal);

end

