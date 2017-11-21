function [ c ] = getObstacleCorner( o, type )
%UpperLeftCorner Get upper-left corner of obstacle
%   Get upper-left corner of obstacle

    switch lower(type)
        case 'upper-left'
            x = getObstacleCoord(o, 'min_x');
            t = getObstacleCoord(o, 'max_t');
        case 'lower-right'
            x = getObstacleCoord(o, 'max_x');
            t = getObstacleCoord(o, 'min_t');
    end
    
    c = [x t];

end

