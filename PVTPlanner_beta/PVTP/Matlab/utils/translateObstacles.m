function [ O_translated ] = translateObstacles( x_offset, t_offset, O )
%translateObstacles Translate a set of obstacles
%   Translate a set of obstacles

    O_translated = [];
    s_O = size(O);
    for i=1:s_O(1, 1)
        
        o = O(i, :);
        o_translated = o + [x_offset x_offset t_offset t_offset];
        O_translated = cat(1, O_translated, o_translated);
        
    end

end

