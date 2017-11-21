function [ S cnt ] = Propagate( p0, p2, V_i, S, O, P, margins )
%Propagate Summary of this function goes here
%   Detailed explanation goes here

    cnt = 0;

    DRAW_TRAJECTORIES = 1;

    % Check reachability
    p_start = [getStateCoord(p0, 'x') getStateCoord(p0, 't')];
    p_goal = [getStateCoord(p2, 'x') getStateCoord(p2, 't')];
    
    [UB LB] = nextReachableSet2(p_start, p_goal, V_i, margins);
    if isempty(UB) || isempty(LB)
        return;
    end
    
    % Get trajectory signatures
    BL = Homotopic(LB, O, P, margins);
    BU = Homotopic(UB, O, P, margins);
    
    if isempty(BL) && isempty(BU)
        return;
    end
    
    %
    % Add velocity intervals
    %
    
    % If an interval is reachable
    if isequal(BL, BU)
        
        S(end+1).Interval = [getStateCoord(LB(end, :), 'v') getStateCoord(UB(end, :), 'v')];
        S(end).B = BL;
        S(end).traj1 = UB;
        S(end).traj2 = LB;
        if DRAW_TRAJECTORIES
            drawTrajectory(UB)
            drawTrajectory(LB)
        end
        
        cnt = 2;
        
    % If singletons only reachable
    else
        
        if ~isempty(BL)
            S(end+1).Interval = [getStateCoord(LB(end, :), 'v') getStateCoord(LB(end, :), 'v')];
            S(end).B = BL;
        S(end).traj1 = LB;
        S(end).traj2 = LB;
            if DRAW_TRAJECTORIES
                drawTrajectory(LB)
            end
            
        end
        
        if ~isempty(BU)
            S(end+1).Interval = [getStateCoord(UB(end, :), 'v') getStateCoord(UB(end, :), 'v')];
            S(end).B = BU;
        S(end).traj1 = UB;
        S(end).traj2 = UB;
            if DRAW_TRAJECTORIES
                drawTrajectory(UB)
            end
            
        end
        
        cnt = 1;
    end
    
    %[V_i p0 p2]
    %UB
    %LB
    
end

