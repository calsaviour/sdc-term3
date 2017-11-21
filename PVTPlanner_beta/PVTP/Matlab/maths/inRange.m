function [ b ] = inRange( val, range, margins )
%inRange Whether a given value is in a given range
%   Whether a given value is in a given range

    b = 0;

    r_max = max(range);
    r_min = min(range);
    if NumCompare(val, r_max, 'le', margins)...
            && NumCompare(val, r_min, 'ge', margins)
        b = 1;
    end

end

