function [ num ] = clipToZero( num, margins )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    if NumCompare(abs(num), 0, 'le', margins)
        num = 0;
    end

end

