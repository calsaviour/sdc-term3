function [ delta_t ] = timeFromAVX( a, v, delta_x, margins )
%timeFromAVX Time given acc, velocity, and distance
%   Time given acc, velocity, and distance

    %if a == 0
    if NumCompare(a, 0, 'eq', margins)
        delta_t = delta_x / v;
    else
        delta_t = (-v + sqrt(v^2 + 2*a*delta_x)) / a;
    end

end

