function [ UB, LB ] = nextReachableSet2( p0, p2, V_i, margins )
%nextReachableSet2 Calculate feasible bounding trajectories from p1 to p2
%   Calculate feasible bounding trajectories from p1 to p2

    SHOW_COMMENTS = 0;

    % The bounding trajectories
    UB = [];
    LB = [];
    
    % Enforce monotonicity constriants
    if ~isReachable(p0, p2, margins)
        if SHOW_COMMENTS
            display('Not reachable: Monotonicity violation.');
        end
        return;
    end

    % Get the possible set of incoming velocities
    %V_i
    i_V = FindInitialVelocityRange(p0, p2, margins);
    if isempty(i_V)
        if SHOW_COMMENTS
            display('Not reachable: No dynamically feasible initial velocities.');
        end
        return;
    end
    
    % Intersect the possible set with the actual set
    V_i = intersectRange(V_i, i_V, margins);
    if isempty(V_i)
        if SHOW_COMMENTS
            display('Not reachable: Empty set of intersected initial velocities.');
        end
        return;
    end
    
    % Use the set of feasible incoming velocities to calculate bounds
    s_i_min = [p0 min(V_i)];
    s_i_max = [p0 max(V_i)];
    UB_min = UpperBoundingStates(s_i_min, p2, margins);
    UB_max = UpperBoundingStates(s_i_max, p2, margins);
    LB_min = LowerBoundingStates(s_i_min, p2, margins);
    LB_max = LowerBoundingStates(s_i_max, p2, margins);
    
    % Exiting after this test should never be triggered
    if isempty(UB_min) || isempty(UB_max) || isempty(LB_min) || isempty(LB_max)
        %V_i
        %[p0 p2]
        %UB_min
        %UB_max
        %LB_min
        %LB_max
        display('Invalid range error in nextReachableSet2; probably a numeric issue.');
        return;
    end
    
    % Figure the correct bounds
    UB_min_v2 = getStateCoord(UB_min(end, :), 'v');
    UB_max_v2 = getStateCoord(UB_max(end, :), 'v');
    if NumCompare(UB_min_v2, UB_max_v2, 'ge', margins)
        UB = UB_min;
    else
        UB = UB_max;
    end
    
    LB_min_v2 = getStateCoord(LB_min(end, :), 'v');
    LB_max_v2 = getStateCoord(LB_max(end, :), 'v');
    if NumCompare(LB_min_v2, LB_max_v2, 'lt', margins)
        LB = LB_min;
    else
        LB = LB_max;
    end

end

