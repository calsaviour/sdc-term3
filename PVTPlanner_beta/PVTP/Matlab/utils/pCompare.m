function [ b ] = pCompare( p1, p2, margins )
%pCompare Compare two points for equality
%   Compare two points for equality

    x1 = getStateCoord(p1, 'x');
    t1 = getStateCoord(p1, 't');
    x2 = getStateCoord(p2, 'x');
    t2 = getStateCoord(p2, 't');
    
    b = NumCompare(x1, x2, 'eq', margins) && NumCompare(t1, t2, 'eq', margins);

end

