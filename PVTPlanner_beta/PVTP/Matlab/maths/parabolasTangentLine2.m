function [ p1 p2 ] = parabolasTangentLine2( v1, v_i, a1, a2, p_goal, margins )
%parabolasTangentLine2 Find the points describing a line tangent to two parabolas
%   Find the points describing a line tangent to two parabolas where the final
%   parabola has a2, and intersects p_goal

    x2 = getStateCoord(p_goal, 'x');
    t2 = getStateCoord(p_goal, 't');


    % find intersection point of line to first parabola
    t_i = (v_i - v1) / a1;
    %x_i = v1 * t_i + 0.5 * a1 * t_i^2;
    x_i = motion(0, v1, t_i, a1);

    % find x intercept of tangent line
    x_int = x_i - v_i * t_i;
    
    % find offset from origin of second parabola where the line is tangent
    t_o = v_i / a2;
    %x_o = 0.5 * a2 * (t_o^2);
    x_o = motion(0, 0, t_o, a2);
    
    % find (x_star, t_star), which is the origin of the second parabola
    a = 1;
    b = 2 * ( (v_i/a2) - t2);
    c = (t2^2) - (t_o^2) + 2 * (v_i * t_o + x_int - x2) / a2;
    if  a2 < a1
        %[t_star t_minus] = quadratic(a, b, c, margins)
        [t_plus t_star] = quadratic(a, b, c, margins);
    else
        [t_plus t_star] = quadratic(a, b, c, margins);
    end
    
    if ~isreal(t_star)
        p1 = [];
        p2 = [];
        return;
    end
    
    x_star = x2 - 0.5 * a2 * (t_star - t2)^2;
    
    % find intersection point of line to second parabola
    x_prime = x_star + x_o;
    t_prime = t_star + t_o;
    
    % define intersection points
    p1 = [x_i t_i];
    p2 = [x_prime t_prime];

end

