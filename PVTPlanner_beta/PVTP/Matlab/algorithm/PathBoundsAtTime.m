function [ p_min p_max ] = PathBoundsAtTime( s_i, t_measure, margins )
%PathBoundsAtTime Determine reachable path range at time t_measure
%   Determine reachable path range at time t_measure

    a_min = getMargin(margins, 'acc_min');
    a_max = getMargin(margins, 'acc_max');
    v_min = getMargin(margins, 'vel_min');
    v_max = getMargin(margins, 'vel_max');
    
    x_i = getStateCoord(s_i, 'x');
    v_i = getStateCoord(s_i, 'v');
    
    % Find time to decelerate to 0
    t = (v_min - v_i) / a_min;
    
    % Velocity is 0 after t_measure
    if NumCompare(t, t_measure, 'gt', margins)
        t = t_measure;
    end
    
    % min path length traversed
    p_min = x_i + motion(0, v_i, t, a_min) + motion(0, v_min, t_measure - t, 0);
    
    % max path length traversed
    t = t_measure;
    v_f = v_i + a_max * t;
    
    % Final velocity is greater than v_max
    if NumCompare(v_f, v_max, 'gt', margins)
        v_f = v_max;
        t = (v_f - v_i) / a_max;
    end
    
    % max path length traversed
    p_max = x_i + motion(0, v_i, t, a_max) + motion(0, v_f, t_measure - t, 0);

end

