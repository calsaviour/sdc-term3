function [ O_r ] = getReachableObstacles( s1, s2, O, margins )
%getReachableObstacles Get the set of obstacle reachable
%   Get the set of obstacle reachable from the bounding trajectory; the
%   bounding trajectory is assumed to be collision-free

    % Set of reachable obstacles
    O_r = [];
    
    % Get the acceleration over this interval
    x_i = getStateCoord(s1, 'x');
    t_i = getStateCoord(s1, 't');
    v_i = getStateCoord(s1, 'v');
    x_f = getStateCoord(s2, 'x');
    t_f = getStateCoord(s2, 't');
    v_f = getStateCoord(s2, 'v');
    a = (v_f - v_i) / (t_f - t_i);
    
    % Loop over all the obstacles, checking each
    s_O = size(O);
    for i=1:s_O(1, 1)

        % Current obstacle
        o = O(i, :);
        
        % Use acceleration to determine time coordinate of trajectory at obstacle edges
        t_min = getObstacleCoord(o, 'min_t');
        t_max = getObstacleCoord(o, 'max_t');
        x_min = getObstacleCoord(o, 'min_x');
        x_max = getObstacleCoord(o, 'max_x');
        delta_x1 = x_min - x_i;
        delta_x2 = x_max - x_i;
        
        % If the obstacle is behind this trajectory, no need to check
        %if delta_x2 < 0
        if NumCompare(delta_x2, 0, 'lt', margins)
            continue;
        end
        
        % If the obstacle is after this trajectory, no need to check
        %if x_min > x_f
        if NumCompare(x_min, x_f, 'gt', margins)
            continue;
        end
        
        % If the obstacle is below this trajectory, no need to check
        %if t_i > t_max
        if NumCompare(t_i, t_max, 'gt', margins)
            continue;
        end
        
        % Check whether the (x1, t1), (x2, t2) of the trajectory intersect      
        t1 = timeFromAVX(a, v_i, delta_x1, margins) + t_i;
        t2 = timeFromAVX(a, v_i, delta_x2, margins) + t_i;
        
        % If the trajectory is strictly below the obstacle, it is "reachable"
        if (NumCompare(t1, t_min, 'lt', margins) && NumCompare(t2, t_min, 'lt', margins))
            O_r = cat(1, O_r, o);
        end
        
    end

end

