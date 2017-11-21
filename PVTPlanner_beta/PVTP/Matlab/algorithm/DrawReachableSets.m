function [ ] = DrawReachableSets( G, margins )
%DrawReachableSets Summary of this function goes here
%   Detailed explanation goes here

    % For each point in G, draw it's reachable sets
    s_G = size(G);
    for i=1:s_G(1, 2)
        
        V = G(i).V;
        s_V = size(V);
        for j=1:s_V(1)
            
            drawVelocityRange(G(i).p, V(j, :), margins);
            
        end
        
    end

end

