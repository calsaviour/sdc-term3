function [ collision ] = CollisionCheckTrajectorySegment( s1, s2, O, margins )
%CollisionCheckTrajectory Perform collision check on trajectory
%   Perform collision check on trajectory, return any obstacle that is
%   intersected

    % Set of in-collision obstacles
    collision = [];
    
    % Get the acceleration over this interval
    x_i = getStateCoord(s1, 'x');
    t_i = getStateCoord(s1, 't');
    v_i = getStateCoord(s1, 'v');
    x_f = getStateCoord(s2, 'x');
    t_f = getStateCoord(s2, 't');
    v_f = getStateCoord(s2, 'v');
    delta_v = v_f - v_i;
    delta_t = t_f - t_i;
    a = delta_v / delta_t;
    
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
        %if x_max <= x_i
        if NumCompare(x_max, x_i, 'le', margins)
            continue;
        end
        
        % If the obstacle is after this trajectory, no need to check
        %if x_min >= x_f
        if NumCompare(x_min, x_f, 'ge', margins)
            continue;
        end
        
        % If the obstacle is below this trajectory, no need to check
        %if t_max <= t_i
        if NumCompare(t_max, t_i, 'le', margins)
            continue;
        end
        
        % If the obstacle is above this trajectory, no need to check
        %if t_min >= t_f
        if NumCompare(t_min, t_f, 'ge', margins)
            continue;
        end
        
        % If the trajectory starts inside the obstacle, set delta_x1 to 0
        %if delta_x1 < 0
        %    delta_x1 = 0;
        %end
        
        %
        % Check trajectory segment endpoints to see whether they're contained in the obstacle
        %
        %if (x_i > x_min) && (x_i < x_max) && (t_i > t_min) && (t_i < t_max)
        if NumCompare(x_i, x_min, 'gt', margins) && NumCompare(x_i, x_max, 'lt', margins)...
                && NumCompare(t_i, t_min, 'gt', margins) && NumCompare(t_i, t_max, 'lt', margins)
            
            % If the condition above holds, we have collision
            collision = cat(1, collision, o);
            continue;
            
        end
        
        %if (x_f > x_min) && (x_f < x_max) && (t_f > t_min) && (t_f < t_max)
        if NumCompare(x_f, x_min, 'gt', margins) && NumCompare(x_f, x_max, 'lt', margins)...
                && NumCompare(t_f, t_min, 'gt', margins) && NumCompare(t_f, t_max, 'lt', margins)
            
            % If the condition above holds, we have collision
            collision = cat(1, collision, o);
            continue;
            
        end
        
        % If this is a wait state
        if NumCompare(a, 0, 'eq', margins) && NumCompare(v_i, 0, 'eq', margins)
            
            % There is necessarily a collision by the above criteria        
            
        % Otherwise, the vehicle is moving
        else

            % Check whether the (x1, t1), (x2, t2) of the trajectory intersect
            t1 = timeFromAVX(a, v_i, delta_x1, margins) + t_i;
            t2 = timeFromAVX(a, v_i, delta_x2, margins) + t_i;
            
            if ~isreal(t2)
                t2 = t_f;
            end
            
            if (NumCompare(t1, t_max, 'ge', margins) && NumCompare(t2, t_max, 'ge', margins))...
                    || (NumCompare(t1, t_min, 'le', margins) && NumCompare(t2, t_min, 'le', margins))
                continue;
            end
            
            %{
            if ~isreal(t2)
                
                if NumCompare(t1, t_max, 'ge', margins)
                    continue;
                end
                
            % Use less/greater than or equal to in order to allow boundary to be intersected
            %if ((t1 >= t_max) && (t2 >= t_max)) || ((t1 <= t_min) && (t2 <= t_min))
            elseif ((t1 >= t_max) && (t2 >= t_max)) || ((t1 <= t_min) && (t2 <= t_min))
                continue;
            end
            %}
            
        end
        
        % If the condition above failed, we have collision
        collision = cat(1, collision, o);
        
    end
    
end

