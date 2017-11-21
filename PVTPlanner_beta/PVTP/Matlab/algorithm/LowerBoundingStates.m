function [ s ] = LowerBoundingStates( s_i, p, margins )
%LowerBoundingStates Given a start state, compute the maximum achievable velocity at p
%   Given a start state, compute the maximum achievable velocity at p

    SHOW_COMMENT = 0;

    % The bounds
    v_min = getMargin(margins, 'vel_min');
    v_max = getMargin(margins, 'vel_max');
    a_min = getMargin(margins, 'acc_min');
    a_max = getMargin(margins, 'acc_max');
    
    x1 = getStateCoord(s_i, 'x');
    t1 = getStateCoord(s_i, 't');
    
    x2 = getStateCoord(p, 'x');
    t2 = getStateCoord(p, 't');
    
    delta_x = x2 - x1;
    delta_t = t2 - t1;
    v1 = getStateCoord(s_i, 'v');
    
    v_avg = delta_x / delta_t;
    
    % Is p even possibly reachable?
    [r s a1] = NRS2NotReachable( s_i, p, margins );
    if r
        s = [];
        a1 = [];
        if SHOW_COMMENT
            display('LB: Unreachable.');
        end
        return;
    else
        s_s = size(s);

        % If only the unique solution exists, we're done
        if s_s(1, 1)>0
            if SHOW_COMMENT
                display('LB: Special case.');
            end
            return;
        end
    end
    
    %
    % CASE 1: Check for arrival at limit
    %

    s = BuildPLPTraj( s_i, [p v_min], margins );
    if ~isempty(s)
        if SHOW_COMMENT
            display('LB: v_min reachable.');
        end
        return;
    end
    
    %
    % CASE 2: Final velocity > v_min with a2 = a_min
    %
    a1 = a_max;
    a2 = a_min;
    v1_tang = v_max;

    [p1 p2] = parabolasTangentLine2( v1, v1_tang, a1, a2, [delta_x delta_t], margins );
    if ~isempty(p1) && ~isempty(p2)
        x1_tang = getStateCoord(p1, 'x');
        t1_tang = getStateCoord(p1, 't');
        x2_tang = getStateCoord(p2, 'x');
        t2_tang = getStateCoord(p2, 't');

        if NumCompare(t1_tang, delta_t, 'le', margins) && NumCompare(t1_tang, 0, 'ge', margins)...
                && NumCompare(t2_tang, delta_t, 'le', margins) && NumCompare(t2_tang, 0, 'ge', margins)...
                && NumCompare(t2_tang, t1_tang, 'ge', margins)...
                && NumCompare(x1_tang, delta_x, 'le', margins) && NumCompare(x1_tang, 0, 'ge', margins)...
                && NumCompare(x2_tang, delta_x, 'le', margins) && NumCompare(x2_tang, 0, 'ge', margins)

            v2 = v1_tang + a2 * (delta_t - t2_tang);

            if NumCompare(v2, v_min, 'ge', margins) && NumCompare(v2, v_max, 'le', margins)

                    % Add wait state
                    s = cat(1, s_i, [x1+x1_tang t1+t1_tang v1_tang]);
                    s = cat(1, s, [x1+x2_tang, t1+t2_tang, v1_tang]);
                    s = cat(1, s, [p v2]);

                    if SHOW_COMMENT
                        display('LB: Case 2a.');
                    end                
                    return;

            end

        end
    end
            
    % Otherwise, adjust v1, t1
    a = 0.5 * (a1 - a2);
    b = delta_t * (a2 - a1);
    c = delta_x - 0.5 * a2 * delta_t^2 - v1 * delta_t;
    [t1_plus t1_tang] = quadratic(a, b, c, margins);
    
    % Validate time
    if isreal(t1_tang) && NumCompare(t1_tang, 0, 'ge', margins) && NumCompare(t1_tang, delta_t, 'le', margins)

        %x1_tang = v1 * t1_tang + 0.5 * a1 * t1_tang^2;
        x1_tang = motion(0, v1, t1_tang, a1);
        v_tang = v1 + a1 * t1_tang;
        t_star = ((a2 - a1) * t1_tang - v1) / a2;
        v2 = a2 * (delta_t - t_star);
        
        if NumCompare(v2, v_min, 'ge', margins)...
                && NumCompare(v2, v_max, 'le', margins)...
                && NumCompare(v_tang, v_min, 'ge', margins)...
                && NumCompare(v_tang, v_max, 'le', margins)

            %s = cat(1, s_i, [x1 t1 v1]);
            %s = cat(1, s, [p v2]);
            
            s = cat(1, s_i, [x1+x1_tang t1+t1_tang v_tang]);
            s = cat(1, s, [p v2]);
            
            if SHOW_COMMENT
                display('LB: Case 2b.');
            end
            return;
            
        end
    end
    
    % Otherwise, there is no final P curve
    if v1 > v_avg
        a1 = a_min;
    else
        a1 = a_max;
    end
    
    a = 0.5 * a1;
    b = -a1 * delta_t;
    c = delta_x - v1 * delta_t;
    [t1_plus t1_minus] = quadratic(a, b, c, margins);
    
    if a1 < 0
        t1_tang = t1_plus;
    else
        t1_tang = t1_minus;
    end
    
    if NumCompare(t1_tang, delta_t, 'le', margins) && NumCompare(t1_tang, 0, 'ge', margins)
        
        v_tang = v1 + a1 * t1_tang;
        
        if NumCompare(v_tang, v_min, 'ge', margins) && NumCompare(v_tang, v_max, 'le', margins)
            
            % Sanity check
            x_tang = motion(0, v1, t1_tang, a1);
            x_lin = v_tang * (delta_t - t1_tang);
            if NumCompare(x_tang+x_lin, delta_x, 'eq', margins)
                
                if SHOW_COMMENT
                    display('LB: Case 3');
                end
                s = cat(1, s_i, [x1+x_tang t1+t1_tang v_tang]);
                s = cat(1, s, [p v_tang]);
                return;
                
            end
            
        end
        
    end
    

    % Everything should be handled by here; if not, check border cases.
    %if SHOW_COMMENT
        display('Unexpected lower-bound case. Re-examine algorithm.');
        s_i
        p
    %end
    
end

