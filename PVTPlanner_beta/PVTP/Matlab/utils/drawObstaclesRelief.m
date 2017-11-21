function [ count ] = drawObstaclesRelief( O, margins )
%drawObtacles Draw a set of obstacles to a figure
%   Draw a set of obstacles to the current figure

    time_limit = getMargin(margins, 'time_limit');
    x_limit = getMargin(margins, 'x_limit');

    daspect([1,1,1]);
    xlim([0, x_limit]);
    ylim([0, time_limit]);
    
    s_O = size(O);
    count = 0;
    for i=1:s_O(1, 1)
        if isReachable([0 0], getObstacleCorner(O(i, :), 'lower-right'), margins)...
                || isReachable([0 0], getObstacleCorner(O(i, :), 'upper-left'), margins)
            drawObstacleRelief(O(i, :));
            hold on;
            count = count + 1;
        end
    end

end

