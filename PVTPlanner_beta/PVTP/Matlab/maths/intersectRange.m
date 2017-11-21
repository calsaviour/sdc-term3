function [ range_new ] = intersectRange( range, bounds, margins )
%intersectRange Intersect a given acceleration range with the bounds
%   Intersect a given acceleration range with the bounds

    if isnan(range(1, 1)) || isnan(range(1, 2))
        range_new = [];
        return;
    end

    range_min = max([range(1, 1) bounds(1, 1)]);
    range_max = min([range(1, 2) bounds(1, 2)]);
    
    %if range_min > range_max
    if NumCompare(range_min, range_max, 'gt', margins)
        range_new = [];
    else
        range_new = [range_min range_max];
    end

end

