function [ Field_List ] = GenerateObstacleFields( O, O_list, Field_List, margins )
%GenerateObstacleFields Enumerate possible paths for H classes to take
%   Enumerate possible paths for H classes to take

    % BASE CASE: Run out of obstacles
    if isempty(O)
        Field_List(end+1).O = O_list;
        return;
    end
    
    % Grow the obstacles in these directions
    x_limit_pos = getMargin(margins, 'x_limit') + 1;
    x_limit_neg = -1;
    t_limit_pos = getMargin(margins, 't_limit') + 1;
    t_limit_neg = -1;
    
    % Mask off upper-left
    o = O(1, :);
    o = setObstacleCoord(o, 'min_x', x_limit_neg);
    o = setObstacleCoord(o, 'max_t', t_limit_pos);
    
    % Add to new list
    O_list(end+1, :) = o;
    
    % Recurse down
    Field_List = GenerateObstacleFields(O(2:end, :), O_list, Field_List, margins);
    
    % Mask off lower-right
    o = O(1, :);
    o = setObstacleCoord(o, 'max_x', x_limit_pos);
    o = setObstacleCoord(o, 'min_t', t_limit_neg);
    
    % Add to new list
    O_list(end, :) = o;
    
    % Recurse down
    Field_List = GenerateObstacleFields(O(2:end, :), O_list, Field_List, margins);
    
end

