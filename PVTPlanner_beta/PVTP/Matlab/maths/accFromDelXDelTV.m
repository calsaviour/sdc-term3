function [ a ] = accFromDelXDelTV( delta_x, delta_t, v )
%accFromDelXDelTV Get acceleration given change in distance, time, and a v
%   Get acceleration given change in distance, time, and a velocity

    a = (2 * delta_x / delta_t^2) - (2 * v / delta_t);

end

