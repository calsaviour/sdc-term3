function [ r, s, a1 ] = NRS2NotReachable( s_i, p, margins )
%NRS2Reachable Quick test for ONLY unreachability
%   Quick test for only unreachability; this doesn't not necessarily guarantee
%   reachability if it returns false. This only guarantees that a point is
%   unreachable if it returns true. This also finds border cases where
%   the only reachable trajectory is one of max or min acceleration; if one
%   of those is detected, it is also returned.

    SHOW_COMMENTS = 0;

    r = 0;
    s = [];
    a1 = [];

    % Some arbitrary bounds
    v_max = getMargin(margins, 'vel_max');
    v_min = getMargin(margins, 'vel_min');
    a_min = getMargin(margins, 'acc_min');
    a_max = getMargin(margins, 'acc_max');
    
    x1 = getStateCoord(s_i, 'x');
    t1 = getStateCoord(s_i, 't');
    
    % There's some abiguous usage of x1 / t1, so introduce these
    x0 = x1;
    t0 = t1;
    
    x2 = getStateCoord(p, 'x');
    t2 = getStateCoord(p, 't');
    
    x = x2 - x1;
    t = t2 - t1;
    v0 = getStateCoord(s_i, 'v');
    
    % Average velocity required over interval
    v_avg = x / t;
    
    % If the average velocity is outside the range of acceptable
    % velocities, there is no solution
    if NumCompare(v_avg, v_min, 'lt', margins) || NumCompare(v_avg, v_max, 'gt', margins)
        r = 1;
        if SHOW_COMMENTS
            display('Average velocity violates velocity constraints. Unreachable');
            display(['v_avg: ', num2str(v_avg), ' [', num2str(v_min), ', ', num2str(v_max), ']']);
        end
        return;
    end
    
    % If deceleration is necessary, make sure it can feasibly happen
    if NumCompare(v0, v_avg, 'gt', margins)
        
        t_inf = -v0 / a_min;
        x_inf = motion(0, v0, t_inf, a_min);
        %[t t_inf]
        %[x x_inf]
        % If the inflection point occurs within the path limit and before the time limit, good
        if NumCompare(x_inf, x, 'le', margins)
            return;
        end
        
        % if the inflection point occurs after the path limit, but within the time limit, no sol
        if NumCompare(x_inf, x, 'gt', margins) && NumCompare(t_inf, t, 'le', margins)
            s = [];
            a1 = [];
            r = 1;
            display('Inflection occurs before time limit, after path limit. Unreachable.');
            return;
        end
        
    end

    % otherwise make sure acceleration is feasible
    a_req = 2 * (x - v0 * t) / t^2;
    if NumCompare(a_req, a_max, 'le', margins) && NumCompare(a_req, a_min, 'ge', margins)
        
        return;
        
    else

        % Nothing else is a valid solution            
        s = [];
        a1 = [];
        r = 1;
        if SHOW_COMMENTS
            display('The endpiont is not reachable (2).');
            display(['a_req: ', num2str(a_req), ' [', num2str(a_min), ', ', num2str(a_max), ']']);
        end
        return;
        
    end

end

