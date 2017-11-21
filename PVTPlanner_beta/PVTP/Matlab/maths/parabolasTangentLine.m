function [ p1, p2, v, x_int ] = parabolasTangentLine( v0, a1, a2, x_star, t_star, margins )
%parabolasTangentLine Find slope and intercept of line tangent to two parabolas
%   Find slope and intercept of line tangent to two parabolas, where one is
%   centered at the origin, and the other at (x_star, t_star).

    % Set up the quadratic
    a = (1 - a2 / a1);
    b = 2 * a2 * t_star + 2 * a2 / a1 * v0;
    c = -(a2 / a1 * v0^2 + 2 * a2 * x_star);
    %a = (a1 - a2);
    %b = 2 * a1 * a2 * t_star + 2 * a2 * v0;
    %c = -(a2 * v0^2 + 2 * a1 * a2 * x_star);
    
    % In the NRS algorithm, only lines of positive slope are of concern
    [v_plus v_minus] = quadratic(a, b, c, margins);
    
    % If a2 < a1, we're looking for v_minus, otherwise, v_plus
    % It should never be the case that a2 = a1; the whole thing rests on
    % that assumption
    if a2 < a1
        v = v_minus;
    else
        v = v_plus;
    end
    
    % If the slope is negative (zero is okay), or the result isn't real,
    % this approach fails
    if ~isreal(v) || NumCompare(v, 0, 'lt', margins)
        p1 = [];
        p2 = [];
        v = [];
        x_int = [];
        return;
    end
    
    % Find the x-intercept
    x_int = -(v0 - v)^2 / (2 * a1);
    
    %
    % (x1, t1) is the intersection point of the parabola centered at the
    % origin, (x2, t2) is the point on the other.
    %
    
    % Find first point
    a = a1;
    b = 2 * (v0 - v);
    c = -2 * x_int;
    
    % We'll use the positive term, but by construction they're both the same
    
    % Quadratic is numerically unstable because of root zero... since root
    % terms is guaranteed to be zero, just calculate manually
    %[t1 t1_minus] = quadratic(a, b, c);
    t1 = -b / (2 * a);
    x1 = motion(0, v0, t1, a1);
    p1 = [x1 t1];
    
    % Find second point
    a = a2;
    b = -2 * (a2 * t_star + v);
    c = 2 * x_star + a2 * t_star^2 - 2 * x_int;
    
    % Quadratic is numerically unstable because of root zero... since root
    % terms is guaranteed to be zero, just calculate manually
    %[t2 t2_minus] = quadratic(a, b, c);
    t2 = -b / (2 * a);
    x2 = motion(x_star, 0, t_star-t2, a2);
    p2 = [x2 t2];

end

