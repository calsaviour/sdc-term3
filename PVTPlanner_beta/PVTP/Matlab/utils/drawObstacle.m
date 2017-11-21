function [] = drawObstacle( o )
%drawObstacle Draw an obstacle to the current figure
%   Draw an obstacle to the current figure

    line_style = '-';
    edge_color = 'k';
    
    x1 = getObstacleCoord(o, 'min_x');
    x2 = getObstacleCoord(o, 'max_x');
    t1 = getObstacleCoord(o, 'min_t');
    t2 = getObstacleCoord(o, 'max_t');

    rectangle('Position', [x1, t1, x2 - x1, t2 - t1], 'LineWidth', 1, 'LineStyle', line_style, 'EdgeColor', edge_color, 'FaceColor', [.8 .8 .8]);


end

