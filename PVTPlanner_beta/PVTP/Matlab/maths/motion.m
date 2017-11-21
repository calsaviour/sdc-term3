function [ x ] = motion( x_0, v_i, delta_t, a )
%motion Equation of motion
%   Equation of motion

    x = x_0 + v_i * delta_t + .5 * a * (delta_t .* delta_t);

end

