function [ ] = drawTrajectory( traj, color )
%drawTrajectory Draw a trajectory to the current figure
%   Draw a trajectory to the current figure

    if (nargin < 2) || isempty(color)
        color = 'b';
    end

    s_traj = size(traj);
    
    % If it's empty, or there's only one state, ignore it
    if s_traj(1, 1)<2
        return;
    end
    
    s1 = traj(1, :);
    for i=2:s_traj(1, 1)
        
        s2 = traj(i, :);
        drawTrajectorySegment(s1, s2, color);
        hold on;
        s1 = s2;
        
    end

end

