function [] = ReadG( G )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    s_G = size(G);
    cnt = 0;
    for i=1:s_G(1,2)
        if ~isempty(G(i).V)
            G(i)
            cnt = cnt + 1;
        end
    end
    cnt

end

