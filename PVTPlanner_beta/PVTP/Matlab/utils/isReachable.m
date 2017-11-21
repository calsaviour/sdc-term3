function [ r ] = isReachable( p1, p2, margins )
%isReachable Determine whether p2 is reachable from p1
%   Determine whether p2 is reachable from p1 given monotonically
%   increasing time, non-decreasing space

    x1 = getStateCoord(p1, 'x');
    t1 = getStateCoord(p1, 't');
    x2 = getStateCoord(p2, 'x');
    t2 = getStateCoord(p2, 't');
    
    x_limit = getMargin(margins, 'x_limit');
    time_limit = getMargin(margins, 'time_limit');
    
    % Time must be strictly greater than, distance not less than
    r = NumCompare(x2, x1, 'ge', margins) && NumCompare(t2, t1, 'gt', margins);
    
    % Make sure we're within time and path length limits
    r = r && NumCompare(t2, time_limit, 'le', margins) && NumCompare(x2, x_limit, 'le', margins);

end

