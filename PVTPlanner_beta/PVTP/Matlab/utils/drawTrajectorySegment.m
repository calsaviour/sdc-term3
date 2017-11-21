function [ ] = drawTrajectorySegment( s1, s2, color )
%drawTrajectorySegment Draw a leg of a trajectory to the current figure
%   Draw a leg of a trajectory to the current figure

    % Start point
    x1 = getStateCoord(s1, 'x');
    t1 = getStateCoord(s1, 't');
    v1 = getStateCoord(s1, 'v');
    
    % End point
    t2 = getStateCoord(s2, 't');
    v2 = getStateCoord(s2, 'v');
    
    delta_t = t2 - t1;
    delta_v = v2 - v1;

    a = delta_v / delta_t;
    
    inc = delta_t / 100;
    start = 0;
    stop = delta_t;

    t_test = start:inc:stop;
    x_test = motion(x1, v1, t_test, a);

    if strcmp(color, 'r')
        h1 = plot(x_test, t_test+t1, 'Color', [0.847 0.161 0], 'LineStyle', '--', 'LineWidth', 2);
        %h1 = plot(x_test, t_test+t1, 'Color', [0.847 0.161 0]);
        hasbehavior( h1, 'legend', false );
    elseif strcmp(color, 'b')
        %h1 = plot(x_test, t_test+t1, 'Color', [0.75 0.75 1]);
        h1 = plot(x_test, t_test+t1, 'Color', [0.75 0.75 1], 'LineWidth', 2);
        hasbehavior( h1, 'legend', false );
    else
        h1 = plot(x_test, t_test+t1, ['-', color]);
        hasbehavior( h1, 'legend', false );
    end

end

