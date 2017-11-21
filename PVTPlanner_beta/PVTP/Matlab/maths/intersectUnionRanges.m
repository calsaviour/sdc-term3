function [ r ] = intersectUnionRanges( range, ranges, margins )
%intersectUnionRanges Intersect a range with a set of ranges
%   Intersect a range with a set of ranges

    % Result set
    r = [];

    s_ranges = size(ranges);
    for i=1:s_ranges(1, 1)
        
        % Current range
        cur_range = ranges(i, :);
        
        % Intersect current range
        intersected = intersectRange(range, cur_range, margins);
        
        % Add to result set
        r = cat(1, r, intersected);
        
    end

end

