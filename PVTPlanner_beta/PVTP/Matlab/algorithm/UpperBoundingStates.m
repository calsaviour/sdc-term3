function [ s ] = UpperBoundingStates( s_i, p, margins )
%UpperBoundingStates Given a start state, compute the maximum achievable velocity at p
%   Given a start state, compute the maximum achievable velocity at p

    SHOW_COMMENT = 0;

    % The bounds
    v_min = getMargin(margins, 'vel_min');
    v_max = getMargin(margins, 'vel_max');
    a_min = getMargin(margins, 'acc_min');
    a_max = getMargin(margins, 'acc_max');
    
    x1 = getStateCoord(s_i, 'x');
    t1 = getStateCoord(s_i, 't');
    
    % There's some abiguous usage of x1 / t1, so introduce these
    x0 = x1;
    t0 = t1;
    
    x2 = getStateCoord(p, 'x');
    t2 = getStateCoord(p, 't');
    
    x = x2 - x1;
    t = t2 - t1;
    v0 = getStateCoord(s_i, 'v');
    
    % Is p even possibly reachable?
    [r s a1] = NRS2NotReachable( s_i, p, margins );
    if r
        s = [];
        a1 = [];
        if SHOW_COMMENT
            display('UB: Unreachable.');
        end
        return;
    else
        s_s = size(s);
        
        % If only the unique solution exists, we're done
        if s_s(1, 1)>1
            if SHOW_COMMENT
                display('UB: Special Case.');
            end
            return;
        end
    end
  
    %
    % CASE 1: Initial and final curves are separate or tangent, final velocity
    % of v_max. Find tangency.
    %
    
    s = BuildPLPTraj( s_i, [p v_max], margins );
    if ~isempty(s)
        if SHOW_COMMENT
            display('UB: v_max reachable.');
        end
        return;
    end

    %
    % CASE 2: Linear segment
    %
    
    % Final velocity less than maximum velocity with v1 = v_min
    a1 = a_min;
    a2 = a_max;
    v1_tang = v_min;

    %[v0 v1_tang a1 a2 x t]
    [p1 p2] = parabolasTangentLine2( v0, v1_tang, a1, a2, [x t], margins );
    if ~isempty(p1) && ~isempty(p2)
        x1_tang = getStateCoord(p1, 'x');
        t1_tang = getStateCoord(p1, 't');
        x2_tang = getStateCoord(p2, 'x');
        t2_tang = getStateCoord(p2, 't');
        if NumCompare(t1_tang, t, 'le', margins) && NumCompare(t1_tang, 0, 'ge', margins)...
                && NumCompare(t2_tang, t, 'le', margins) && NumCompare(t2_tang, 0, 'ge', margins)...
                && NumCompare(t2_tang, t1_tang, 'ge', margins)...
                && NumCompare(x1_tang, x, 'le', margins) && NumCompare(x1_tang, 0, 'ge', margins)...
                && NumCompare(x2_tang, x, 'le', margins) && NumCompare(x2_tang, 0, 'ge', margins)

            v2 = v1_tang + a2 * (t - t2_tang);

            if NumCompare(v2, v_min, 'ge', margins) && NumCompare(v2, v_max, 'le', margins)

                    % Add wait state
                    s = cat(1, s_i, [x1+x1_tang t1+t1_tang v1_tang]);
                    s = cat(1, s, [x1+x2_tang, t1+t2_tang, v1_tang]);
                    s = cat(1, s, [p v2]);

                    if SHOW_COMMENT
                        display('UB: Case 2a.');
                    end                
                    return;

            end

        end
    end
    
    % Otherwise, adjust v1, t1
    a = a1 - a2;
    b = 2 * t * (a2 - a1);
    c = 2 * x - a2 * t^2 - 2 * v0 * t;
    [t1 t1_minus] = quadratic(a, b, c, margins);
    
    if t1 < 0
        t1 = clipToZero(t1, margins);
    end

    % Validate time
    if isreal(t1) && NumCompare(t1, 0, 'ge', margins) && NumCompare(t1, t, 'le', margins)

        x1 = v0 * t1 + 0.5 * a1 * t1^2;
        v1 = v0 + a1 * t1;
        t_star = ((a2 - a1) * t1 - v0) / a2;
        v2 = a2 * (t - t_star);
       
        if NumCompare(v2, v_max, 'le', margins)...
                && NumCompare(v2, v_min, 'ge', margins)

            %s = cat(1, s_i, [x1 t1 v1]);
            %s = cat(1, s, [p v2]);
            
            s = cat(1, s_i, [x1+x0 t1+t0 v1]);
            s = cat(1, s, [p v2]);
            
            if SHOW_COMMENT
                display('UB: Case 3.');
            end
            return;
            
        end
    end
    
    % Everything should be handled by here; if not, check border cases.
    %if SHOW_COMMENT
        display('Unexpected upper-bound case. Re-examine algorithm.');
        s_i
        p
    %end
    
end

