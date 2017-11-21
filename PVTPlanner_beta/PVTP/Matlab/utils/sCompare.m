function [ b ] = sCompare( s1, s2, margins )
% sCompare Compare two states
%   Compare two states

    b = 0;

    % First, check points
    p1 = [getStateCoord(s1, 'x') getStateCoord(s1, 't')];
    p2 = [getStateCoord(s2, 'x') getStateCoord(s2, 't')];
    if pCompare(p1, p2, margins)
    
        % If the points are the same, compare velocities
        v1 = getStateCoord(s1, 'v');
        v2 = getStateCoord(s2, 'v');
        if NumCompare(v1, v2, 'eq', margins)
            b = 1;
            return;
        end
        
    end
    
end

