function [ T ] = GetOptimalTrajectory( G, Goal, O, margins )
%GetOptimalTrajectory Summary of this function goes here
%   Detailed explanation goes here

    SHOW_COMMENTS = 0;

    % Container for the final trajectory
    T = [];

    % Nothing to do if Goal is empty; G can be empty if there are no obstacles
    if isempty(Goal)
        return;
    end
    
    % Get specified accuracy
    accuracy = getMargin(margins, 'accuracy');

    % Initial segment to test
    UB_t = getStateCoord(Goal(1).UB(end, :), 't');
    LB_t = getStateCoord(Goal(1).LB(end, :), 't');
    if NumCompare(LB_t, UB_t, 'lt', margins)
        cur_seg = Goal(1).LB;
        cur_seg_t = LB_t;
    else
        cur_seg = Goal(1).UB;
        cur_seg_t = UB_t;
    end

    % Find the fastest goal connection
    index = 1;
    s_Goal = size(Goal);
    for i=1:s_Goal(1, 2)
        
        UB_t = getStateCoord(Goal(i).UB(end, :), 't');
        LB_t = getStateCoord(Goal(i).LB(end, :), 't');
        
        if NumCompare(LB_t, cur_seg_t, 'lt', margins)
            cur_seg = Goal(i).LB;
            cur_seg_t = LB_t;
        end
        
        if NumCompare(UB_t, cur_seg_t, 'lt', margins)
            cur_seg = Goal(i).UB;
            cur_seg_t = UB_t;
        end
        
    end
    
    %
    % cur_seg contains the final segment that connects fastest to goal
    %

    % Append the final leg to the trajectory being built
    T = cat(1, T, cur_seg);
    
    % Get start state of current segment
    c_x = getStateCoord(cur_seg(1, :), 'x');
    c_t = getStateCoord(cur_seg(1, :), 't');
    c_v = getStateCoord(cur_seg(1, :), 'v');
    
    % We're often working right at the bounds, so numerical errors happen
    c_v = truncdec(c_v, accuracy, 0);
    
    % If we happen to be at the start, we're done
    if pCompare([c_x c_t], [0 0], margins)
        return;
    end
    
    % Run this loop until the trajectory arrives at the start
    at_start = 0;
    while ~at_start
    
        % Look through the rest of the nodes to find a way back to the start
        s_G = size(G);
        for i=1:s_G(1, 2)

            % An empty velocity interval means it's not on any feasible trajectory
            if isempty(G(i).V)
                continue;
            end

            % Candidate point
            cand_p = G(i).p;
            cand_x = getStateCoord(cand_p, 'x');
            cand_t = getStateCoord(cand_p, 't');

            % If this point violates monotonicity constraints, ignore it
            if ~isReachable([cand_x cand_t], [c_x c_t], margins)
                
                if SHOW_COMMENTS
                    [cand_x cand_t]
                    [c_x c_t]
                    display('Monotonically unreachable.');
                end
                continue;
            end

            % Mirror the problem, find route with nextReachableSet
            delta_x = c_x - cand_x;
            delta_t = c_t - cand_t;
            tmp_x = c_x + delta_x;
            tmp_t = c_t + delta_t;
            
            % mirror acceleration range
            tmp_margins = setMargin([-getMargin(margins, 'acc_max') -getMargin(margins, 'acc_min')], margins, 'acc');
            
            [UB LB] = nextReachableSet2([c_x c_t], [tmp_x tmp_t], [c_v c_v], tmp_margins);
            if isempty(UB) || isempty(LB)
                if SHOW_COMMENTS
                    [c_x c_t c_v]
                    [tmp_x tmp_t]
                    display('NRS2 Unreachable.');
                end
                continue;            
            end

            % Set of velocities reachable at the candidate node
            cand_V_i = [getStateCoord(LB(end, :), 'v') getStateCoord(UB(end, :), 'v')];
            V_i = intersectUnionRanges(cand_V_i, G(i).V, margins);
            if isempty(V_i)

                % Get specified accuracy
                accuracy = getMargin(margins, 'accuracy');
                
                % Try to resolve any numeric issues
                range_min = max([cand_V_i(1, 1) G(i).V(1, 1)]);
                range_max = min([cand_V_i(1, 2) G(i).V(1, 2)]);
                range_diff = abs(range_max - range_min);
 
                % If it's still empty, probably no good
                if NumCompare(range_min, range_max, 'gt', margins)
                    if SHOW_COMMENTS
                        cand_V_i
                        G(i).V
                        [c_x c_t c_v]
                        [tmp_x tmp_t]
                        display('Empty velocity intersection.');
                    end
                    continue;
                end
                
                if NumCompareNoRound(range_diff, 0, 'eq', margins)
                    V_i = [];
                else
                    V_i = [range_min range_max];
                end
                
                % If it's still empty, probably no good
                if isempty(V_i)
                    if SHOW_COMMENTS
                        display('Empty velocity intersection.');
                    end
                    continue;
                end
            end

            % Pick an initial velocity from range
            s_V_i = size(V_i);
            for i=1:s_V_i(1, 1)
                v0 = max(V_i(i,:));
                v0_other = min(V_i(i,:));
                traj = BuildPLPTraj([cand_x cand_t v0], [c_x c_t c_v], margins);
                if isempty(traj)
                    v0 = v0_other;
                    v0_other = max(V_i);
                    traj = BuildPLPTraj([cand_x cand_t v0], [c_x c_t c_v], margins);
                end
                if ~isempty(traj)
                    break;
                end
            end
            
            % This should never happend
            if isempty(traj)
                display('GetOptimalTrajectory: Failed building a trajectory.');                 
                continue;
            end
            
            % Collision check trajectory
            c = NRS2CollisionCheckTrajectory(traj, O, margins);
            if ~isempty(c)

                % Try other end of V_i
                v0 = v0_other;
                traj = BuildPLPTraj([cand_x cand_t v0], [c_x c_t c_v], margins);
                if isempty(traj)
                    if SHOW_COMMENTS
                        display('GetOptimalTrajectory: Failed to find collision-free trajectory.');
                    end
                    continue;
                end
                c = NRS2CollisionCheckTrajectory(traj, O, margins);
                if ~isempty(c)
                    if SHOW_COMMENTS
                        display('In collision.');
                    end
                    continue;
                end

            end

            % At this point there's a feasible trajectory segment; add it
            T = cat(1, traj, T);

            % Move to candidate point
            cur_seg = traj;
            c_x = getStateCoord(cur_seg(1, :), 'x');
            c_t = getStateCoord(cur_seg(1, :), 't');
            c_v = getStateCoord(cur_seg(1, :), 'v');
            
            % We're often working right at the bounds, so numerical errors happen
            c_v = truncdec(c_v, accuracy, 0);
            
            % Break out to start over looking through nodes
            break;

        end
        
        % Check to see where we are
        if pCompare([c_x c_t], [0 0], margins)
            at_start = 1;
        end
        
    end
    
end

