function [ S cnt ] = PropagateGoal( p0, V_i, V_f, S, O, P, margins )
%PropagateGoal Summary of this function goes here
%   Detailed explanation goes here

    cnt = 0;

    SHOW_COMMENTS = 0;
    DRAW_TRAJECTORIES = 1;

    % Check reachability
    p_start = [getStateCoord(p0, 'x') getStateCoord(p0, 't')];
    
    [UB LB] = GoalConnect( p_start, V_i, V_f, O, margins );
    if isempty(UB) || isempty(LB)
        if SHOW_COMMENTS
            display('Goal unreachable.');
        end
        return;
    end
    
    % Get trajectory signatures
    BL = Homotopic(LB, O, P, margins);
    BU = Homotopic(UB, O, P, margins);
    if isempty(BL) && isempty(BU)
        if SHOW_COMMENTS
            display('No valid homotopy for path.');
        end
        return;
    end
    
    %
    % Add velocity intervals
    %

    % If an interval is reachable
    if isequal(BL, BU)
        
        S(end+1).B = BL;
        S(end).UB = UB;
        S(end).LB = LB;
        
        if DRAW_TRAJECTORIES
            drawTrajectory(S(end).UB);
            drawTrajectory(S(end).LB);
        end
        
        cnt = 2;
        
    else
        
        cnt = 1;
        
        % Both are reachable, but from different homotopic classes
        if ~isempty(BL) && ~isempty(BU)
            
            % Top class
            S(end+1).B = BU;
            S(end).UB = UB;
            S(end).LB = UB;
            
            if DRAW_TRAJECTORIES
                drawTrajectory(S(end).UB);
            end
            
            % Bottom class
            S(end+1).B = BL;
            S(end).UB = LB;
            S(end).LB = LB;
            
            if DRAW_TRAJECTORIES
                drawTrajectory(S(end).LB);
            end
            
            cnt = 2;
            
        % The bottom was reachable
        elseif ~isempty(BL)
            
            % Bottom class
            S(end+1).B = BL;
            S(end).UB = LB;
            S(end).LB = LB;
            
            if DRAW_TRAJECTORIES
                drawTrajectory(S(end).LB);
            end
            
        % The top was reachable
        else
            
            % Top class
            S(end+1).B = BU;
            S(end).UB = UB;
            S(end).LB = UB;
            
            if DRAW_TRAJECTORIES
                drawTrajectory(S(end).UB);
            end
            
        end
        
    end
    
end
