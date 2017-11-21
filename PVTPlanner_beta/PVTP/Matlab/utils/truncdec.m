function [ n ] = truncdec( n, places, rnd )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here

    m = 10^places;
    
    if rnd
        n = round(n * m);
    else
        n = n * m;
    end
    
    n = n / m;

end

