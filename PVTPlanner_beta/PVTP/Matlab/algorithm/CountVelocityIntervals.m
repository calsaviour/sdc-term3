function [ num_intervals ] = CountVelocityIntervals( G )
%CountVelocityIntervals Summary of this function goes here
%   Detailed explanation goes here

    num_intervals = 0;

    s_G = size(G);
    for i=1:s_G(1, 2)
        
        if ~isempty(G(i).V)
            s_V = size(G(i).V);
            num_intervals = num_intervals + s_V(1, 1);
        end
        
    end

end

