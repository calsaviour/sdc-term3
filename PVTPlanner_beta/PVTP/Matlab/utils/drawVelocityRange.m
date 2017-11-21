function [ ] = drawVelocityRange( p, V, margins )
%drawVelocityRange Summary of this function goes here
%   Detailed explanation goes here

    num_lines = 50;
    line_length = 0.5;
    
    min_v = min(V);
    max_v = max(V);
    
    p_x = getStateCoord(p, 'x');
    p_t = getStateCoord(p, 't');

    % Get angle
    if NumCompare(min_v, 0, 'eq', margins)
        theta = pi / 2;
    else
        theta = atan(1 / min_v);
    end

    % Get length of legs of triangle
    t = line_length * sin(theta);
    x = line_length * cos(theta);

    % Draw line
    line([p_x; p_x+x], [p_t; p_t+t], 'Color', 'r');
    
    delta_v = max_v - min_v;
    
    % If this is zero, we're done
    if NumCompare(delta_v, 0, 'eq', margins)
        return;
    end
    
    v_inc = delta_v / num_lines;
    
    for v=min_v:v_inc:max_v
        
        % Get angle
        if NumCompare(v, 0, 'eq', margins)
            theta = pi / 2;
        else
            theta = atan(1 / v);
        end
        
        % Get length of legs of triangle
        t = line_length * sin(theta);
        x = line_length * cos(theta);
        
        % Draw line
        line([p_x; p_x+x], [p_t; p_t+t], 'Color', 'r');
        
    end

end

