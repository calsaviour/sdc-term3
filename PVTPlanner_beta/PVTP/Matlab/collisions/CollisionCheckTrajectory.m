function [ collision, starts, ends ] = CollisionCheckTrajectory( traj, O, margins )
%CollisionCheckTrajectory Check a trajectory for collision among obstacles
%   Check a trajectory for collision among obstacles in O

    collision = [];
    starts = [];
    ends = [];
    
    s_traj = size(traj);
    s_O = size(O);
    
    % No need to collision check if there are no obstacles, or the
    % trajectory is empty
    if (s_traj(1, 1) < 2) || (s_O(1, 1) < 1)
        return;
    end
    
    s1 = traj(1, :);
    for i=2:s_traj(1, 1)
        
        s2 = traj(i, :);
        c = CollisionCheckTrajectorySegment(s1, s2, O, margins);
        
        if size(c) > 0
            collision = cat(1, collision, c);
            starts = cat(1, starts, s1);
            ends = cat(1, ends, s2);
        end
        
        s1 = s2;
        
    end
    
end

