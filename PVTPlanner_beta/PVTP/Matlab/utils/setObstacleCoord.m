function [ o ] = setObstacleCoord( o, coord, c )
%setObstacleCoord Set the value of an obstacle coordinate
%   Set the value of an obstacle coordinate


    switch lower(coord)
        case 'min_x'        % Lower X boundary
            o(1, 1) = c;
        case 'max_x'        % Upper X boundary
            o(1, 2) = c;
        case 'min_t'        % Lower T boundary
            o(1, 3) = c;
        case 'max_t'        % Upper T boundary
            o(1, 4) = c;
    end

end

