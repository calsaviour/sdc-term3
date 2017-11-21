function [ UB, LB ] = GoalConnect( p0, V_i, V_f, O, margins )
%GoalConnect Summary of this function goes here
%   Detailed explanation goes here

    SHOW_COMMENTS = 0;

    % Goal position on path, x_limit, and hard time limit, t_limit
    x_limit = getMargin(margins, 'x_limit');
    t_limit = getMargin(margins, 't_limit');
    
    % Dynamic constraints
    v_g_min = min(V_f);
    v_g_max = max(V_f);
    v_min = getMargin(margins, 'vel_min');
    v_max = getMargin(margins, 'vel_max');
    a_max = getMargin(margins, 'acc_max');
    a_min = getMargin(margins, 'acc_min');
    
    %
    % From the bounds of V_i, calculate the fastest and slowest we can
    % arrive at x_limit while respecting dynamic constraints
    %
    
    v0_min = min(V_i);
    v0_max = max(V_i);
    
    x = getStateCoord(p0, 'x');
    t = getStateCoord(p0, 't');
    
    delta_x = x_limit - x;
    
    UB = [];
    LB = [];
    
    %
    % PART 1: First find absolute velocity limits at goal, then intersect with V_f,
    % then compute bounding trajectories from there
    %
    
    % Max arrival velocity
    a = a_max;
    b = 2 * v0_max;
    c = -2 * delta_x;
    [delta_t t_minus] = quadratic(a, b, c, margins);
    
    % If this violates constrains, goal is not reachable
    if NumCompare(t + delta_t, t_limit, 'gt', margins)
        if SHOW_COMMENTS
            display('Goal unreachable.');
        end
        return;
    end
    
    % Max arrival velocity
    v_f_max = v0_max + a_max * delta_t;
    
    % If the arrival velocity exceeds max velocity, dial it back
    if NumCompare(v_f_max, v_max, 'gt', margins)
        
        % Point at which velocity maxes
        t_i = (v_max - v0_max) / a_max;
        x_i = motion(0, v0_max, t_i, a_max);
        
        % Linear segment to goal
        t_f = (delta_x - x_i) / v_max;
        
        % Total time
        delta_t = t_i + t_f;
        
        % If this exceeds bounds, the goal is unreachable
        if NumCompare(t+delta_t, t_limit, 'gt', margins)
            if SHOW_COMMENTS
                display('Goal unreachable: case 1.');
            end
            return;
        end
        
        % Goal is reachable at max velocity
        v_f_max = v_max;
        
    end
    
    % Min arrival velocity
    [UB_nrs LB_nrs] = nextReachableSet2([x t], [x_limit t_limit], V_i, margins);
    
    % If the time limit cannot be reached, then the trajectory must hit the
    % x_limit below the t_limit with some non-zero velocity; find that time
    if isempty(UB_nrs) || isempty(LB_nrs)
        
        % Try decelerating from v0_min first
        v0 = v0_min;
        
        a = a_min;
        b = 2 * v0;
        c = -2 * delta_x;
        [t1 t_minus] = quadratic(a, b, c, margins);
        if ~isreal(t1) || NumCompare(t1, t_limit, 'gt', margins) || NumCompare(t1, 0, 'lt', margins)
            v0 = v0_max;
            b = 2 * v0;
            [t1 t_minus] = quadratic(a, b, c, margins);
            if ~isreal(t1) || NumCompare(t1, t_limit, 'gt', margins) || NumCompare(t1, 0, 'lt', margins)
                if SHOW_COMMENTS
                    display('Goal unreachable: case 2.');
                end
                return;
            end
        end
        
        % Calculate arrival velocity
        v_f_min = v0 + a_min * t1;
        
    else

        % Min arrival velocity
        v_f_min = getStateCoord(LB_nrs(end, :), 'v');
    
    end
    
    % Velocity range at goal that is dynamically feasible
    V_g = [v_f_min v_f_max];
    
    % Intersect this with desired goal range
    V = intersectRange(V_g, V_f, margins);
    if isempty(V)
        if SHOW_COMMENTS
            display('Goal unreachable: case 3.');
        end
        return;
    end
    
    % Valid goal velocities
    v_f_min = min(V);
    v_f_max = max(V);
    
    % Valid initial velocities
    V_i = [v0_min v0_max];
    
    
    % Now with valid bounds on arrival velocity, compute corresponding trajectories

    %
    % PART 2: Build representative trajectories that arrive at the goal
    % velocity bounds
    %
 
    % Find crossover point with v0 = v0_max, a1 = a_max, a2 = a_min
    v0 = v0_max;
    vf = v_f_max;
    
    UB = BuildGoalPLP(p0, V_i, vf, margins);
    if isempty(UB)
        
        % If there is no feasible trajectory, try with other initial velocity
        v0 = v0_min;
        UB = BuildGoalPLP(p0, V_i, vf, margins);
        if isempty(UB)
            
            % This should never happen
            if isempty(UB)
                display('GoalConnect: Error computing max velocity representative trajectory to goal; probably a numeric issue.');
                return;
            end
            
        end
        
    end

    %
    % Buid a trajectory that arrives at the min goal velocity
    %

    v0 = v0_max;
    vf = v_f_min;

    LB = BuildGoalPLP(p0, V_i, vf, margins);
    if isempty(LB)

        % If there is no feasible trajectory, try with other initial velocity
        v0 = v0_min;
        LB = BuildGoalPLP(p0, V_i, vf, margins);
        if isempty(LB)
            
            % This should never happen
            if isempty(LB)
                display('GoalConnect: Error computing min velocity representative trajectory to goal; probably a numeric issue.');
                return;
            end
            
        end
        
    end
    
end

