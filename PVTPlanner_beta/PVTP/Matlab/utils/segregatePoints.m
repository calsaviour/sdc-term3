function [ UL, LR ] = segregatePoints( O )
%segregatePoints Given a set of obstacles, segregate its corner points
%   Given a set of obstacles, segregate its corner points into upper-left
%   and lower-right sets

    s_O = size(O);
    
    UL = zeros(s_O(1,1), 2);
    LR = zeros(s_O(1,1), 2);
    
    for i=1:s_O(1, 1)
        
        o = O(i, :);
        UL(i, :) = getObstacleCorner(o, 'upper-left');
        LR(i, :) = getObstacleCorner(o, 'lower-right');
        
    end

end

