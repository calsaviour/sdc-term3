function [ c ] = NRS2CollisionCheckTrajectory( t, O, margins )
%NRS2CollisionCheckTrajectory Check a trajectory for collision among O
%   Check a trajectory for collision among set of obstacles O

    %
    % 't' is the set of points that constitute the trajectory, for each
    % segment run the previously defined trajectory segment collision check
    %
    
    c = [];
    s_t = size(t);
    s_O = size(O);
    
    if s_t(1, 1)<2 || s_O(1, 1)==0
        return;
    end
    
    s1 = t(1, :);
    for i=2:s_t(1, 1)
        
        s2 = t(i, :);
        c = cat(1, c, CollisionCheckTrajectorySegment(s1, s2, O, margins));
        
        s1 = s2;
        
    end

end

