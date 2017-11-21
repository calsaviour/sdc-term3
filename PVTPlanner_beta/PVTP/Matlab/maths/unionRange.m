function [ u ] = unionRange( range1, range2, margins )
%unionRange Union two ranges
%   Union two ranges

    r1_min = min(range1);
    r1_max = max(range1);
    r2_min = min(range2);
    r2_max = max(range2);

    % range2 subset range1
    if isSubset(range2, range1, margins)
        u = range1;
        return;
    end
    
    % range1 subset range1
    if isSubset(range1, range2, margins)
        u = range2;
        return;
    end
    
    % range1 -> range2
    if NumCompare(r1_min, r2_min, 'le', margins)...
            && NumCompare(r1_max, r2_min, 'ge', margins)
        u = [r1_min r2_max];
        return;
    end
    
    % range2 -> range1
    if NumCompare(r2_min, r1_min, 'le', margins)...
            && NumCompare(r2_max, r1_min, 'ge', margins)
        u = [r2_min r1_max];
        return;
    end
    
    % Otherwise, they are distinct
    u = [range1; range2];

end

