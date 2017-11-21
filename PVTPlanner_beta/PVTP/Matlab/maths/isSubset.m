function [ b ] = isSubset( range1, range2, margins )
%isSubset Is range1 a subset of range2?
%   Is range1 a subset of range2?

    r1_min = min(range1);
    r1_max = max(range1);
    r2_min = min(range2);
    r2_max = max(range2);
    
    if NumCompare(r2_min, r1_min, 'le', margins)...
            && NumCompare(r2_max, r1_max, 'ge', margins)
        b = 1;
    else
        b = 0;
    end

end

