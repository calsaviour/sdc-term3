function [ c ] = getObstacleCoord( o, coord )
%getObstacleCoord Get a coordinate from an obstacle
%   Get a coordinate from an obstacle

    switch lower(coord)
        case 'min_x'        % Lower X boundary
            c = o(1, 1);
        case 'max_x'        % Upper X boundary
            c = o(1, 2);
        case 'min_t'        % Lower T boundary
            c = o(1, 3);
        case 'max_t'        % Upper T boundary
            c = o(1, 4);
    end


end

