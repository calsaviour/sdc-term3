function [ traj ] = BuildGoalPLP( p0, V_i, v2, margins )
%BuildGoalPLP Summary of this function goes here
%   Detailed explanation goes here

    SHOW_COMMENTS = 0;
    
    traj = [];
    
    % Constraints
    a_max = getMargin(margins, 'acc_max');
    a_min = getMargin(margins, 'acc_min');
    v_max = getMargin(margins, 'vel_max');
    v_min = getMargin(margins, 'vel_min');

    % Goal position on path, x_limit, and hard time limit, t_limit
    x_limit = getMargin(margins, 'x_limit');
    t_limit = getMargin(margins, 't_limit');
    x = getStateCoord(p0, 'x');
    t = getStateCoord(p0, 't');
    
    delta_x = x_limit - x;
    
    v0_min = min(V_i);
    v0_max = max(V_i);

    %
    % Compute bounding trajectories:
    % Assume initial acceleration is a_max, final is a_min. First compute
    % switching time by ignoring velocity constraints along trajectory. If
    % there exists a valid switching time, then check velocity constraint at
    % that time. If it fails, insert a linear segment.
    %

    a1 = a_max;
    a2 = a_min;
    v0 = v0_max;
    a = a1 * a2 - a1^2;
    %b = 2 * a2 * v0 - 2 * a1 * v0;
    b = 2 * v0 * (a2 - a1);
    c = (v2^2) - (v0^2) - 2 * a2 * delta_x;
    [t_plus t1] = quadratic( a, b, c, margins );
    
    % Ensure the switch occurs at valid time; if not adjust initial velocity
    % such that it does
    if NumCompare(t1, 0, 'lt', margins)
        t1 = 0;
        discriminant = (v2^2) - 2 * a2 * delta_x;
        v0 = sqrt( discriminant );
    end
    if ~inRange(v0, V_i, margins)
        if SHOW_COMMENTS
            display('Goal unreachable.');
        end
        return;
    end
    
    v0;

    % Calculate velocity at switching time
    v1 = v0 + a1 * t1;
    
    % If switching velocity is outside bounds, insert linear segment
    if NumCompare(v1, v_max, 'gt', margins)
        if SHOW_COMMENTS
            display('Linear segment case.');
        end
        
        % Build linear segment
        t1 = (v_max - v0) / a1;
        t2 = (v2 - v_max) / a2;
        x2 = v_max * t2 + 0.5 * a2 * (t2^2);
        x1 = motion(0, v0, t1, a1);
        delta_x_linear = delta_x - x1 - x2;
        delta_t_linear = delta_x_linear / v_max;
        
        % Make sure time is valid
        total_time = t + t1 + delta_t_linear + t2;
        if NumCompare(total_time, t_limit, 'gt', margins)
            if SHOW_COMMENTS
                display('Linear segment case: time constraint violated. (1)');
                return;
            end
        end
        
        % If we make it here, we're done
        traj = [
            x                         t                         v0;
            x+x1                      t+t1                      v_max;
            x+x1+delta_x_linear       t+t1+delta_t_linear       v_max;
            x+x1+delta_x_linear+x2    total_time                v2  
            ];
        
    else
        if SHOW_COMMENTS
            display('No linear segment case.');
        end
        
        t2 = (v2 - v1) / a2;
        total_time = t + t1 + t2;
        if NumCompare(total_time, t_limit, 'gt', margins)
            if SHOW_COMMENTS
                display('No linear segment case: time constraint violated. (2)');
                return;
            end
        end
        
        x1 = motion(0, v0, t1, a1);
        x2 = motion(0, v1, t2, a2);
        
        traj = [
            x       t           v0;
            x+x1    t+t1        v1;
            x+x1+x2 total_time  v2
            ];
    end

    %{
    % Find the set of accelerations that work
    a1 = a_min;
    a2 = a_max;
    v_int = v_min;
    v0 = v_max;
    vf = v2;

    a = (a1^2) - a1 * a2;
    b = 2 * a1 * v0 - 2 * a2 * v0;
    c = v0^2 - vf^2 + 2 * a2 * delta_x;
    [t_plus t_minus] = quadratic(a, b, c, margins);
    
    if NumCompare(t_plus, 0, 'ge', margins) && NumCompare(t_minus, 0, 'ge', margins)
        t1 = min([t_plus t_minus]);
    else
        t1 = max([t_plus t_minus]);
    end

    if ~isreal(t1) || NumCompare(t1, t_limit, 'gt', margins)...
        || NumCompare(t1, 0, 'lt', margins)
    
        a1 = a_max;
        a2 = a_min;
        v_int = v_max;

        a = (a1^2) - a1 * a2;
        b = 2 * a1 * v0 - 2 * a2 * v0;
        c = v0^2 - vf^2 + 2 * a2 * delta_x;
        [t_plus t_minus] = quadratic(a, b, c, margins);
        
        if NumCompare(t_plus, 0, 'ge', margins) && NumCompare(t_minus, 0, 'ge', margins)
            t1 = min([t_plus t_minus]);
        else %if NumCompare(t_plus, 0, 'ge', margins) || NumCompare(t_minus, 0, 'ge', margins)
            t1 = max([t_plus t_minus]);
        end
        
    end
    
    % Verify that this does not violate velocity constraints
    v1 = v0 + a1 * t1;
    if NumCompare(v1, v_max, 'gt', margins) || NumCompare(v1, v_min, 'lt', margins)
        
        if SHOW_COMMENTS
            display('BuildGoalPLP: Case 1.');
        end
        
        % Relative point of tangency on final curve
        t2_tang = (vf - v_int) / a2;
        x2_tang = motion(0, v_int, t2_tang, a2);
        
        % Point of tangency on initial curve
        t1_tang = (v_int - v0) / a1;
        x1_tang = motion(0, v0, t1_tang, a1);
        
        % End position of linear segment
        x2_lin = delta_x - x2_tang;
        
        % End time of linear segment
        if NumCompare(v_int, 0, 'eq', margins)
            t2_lin = 0;
        else
            t2_lin = t1_tang + (x2_lin - x1_tang) / v_int;
        end
        
        % Total elapsed time
        delta_t = t2_lin + t2_tang;
        
        % Build states
        traj = [
            x           t           v0;
            x+x1_tang	t+t1_tang   v_int;
            x+x2_lin	t+t2_lin    v_int;
            x_limit     t+delta_t   vf
            ];
        return;
    
    else
        
        if SHOW_COMMENTS
            display('BuildGoalPLP: Case 2.');
        end
        
        % Build trajectory
        x1 = motion(0, v0, t1, a1);
        t2 = (vf - v1) / a2;
        delta_t = t1 + t2;

        if NumCompare(x+x1, x_limit, 'gt', margins)...
                || NumCompare(t+delta_t, t_limit, 'gt', margins)
            if SHOW_COMMENTS
                [x+x1 x_limit]
                [t+delta_t t_limit]
                display('BuildGoalPLP: Unreachable.');
            end
            return;
        end  
        
        traj = [
            x       t           v0;
            x+x1    t+t1        v1;
            x_limit t+delta_t   vf
            ];
        return;
        
    end
    %}
end

