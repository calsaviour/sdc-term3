function [ t ] = BuildPLPTraj( s1, s2, margins )
%BuildPLPTraj Given an initial state and a final state, build a PLP curve
%   Given an initial state and a final state, build a PLP curve connecting
%   the two states

    SHOW_COMMENTS = 0;

    % Dynamic constraints
    a_min = getMargin(margins, 'acc_min');
    a_max = getMargin(margins, 'acc_max');
    v_min = getMargin(margins, 'vel_min');
    v_max = getMargin(margins, 'vel_max');
    v_feasible = [v_min v_max];

    % Initial velocity
    v1 = getStateCoord(s1, 'v');
    
    % Final velocity
    v2 = getStateCoord(s2, 'v');
    
    % Average velocity
    x1 = getStateCoord(s1, 'x');
    t1 = getStateCoord(s1, 't');
    x2 = getStateCoord(s2, 'x');
    t2 = getStateCoord(s2, 't');
    delta_x = x2 - x1;
    delta_t = t2 - t1;
    v_avg = delta_x / delta_t;
    
    % Quick check of feasibility
    if ~inRange(v_avg, v_feasible, margins)
        if SHOW_COMMENTS
            display('Average velocity makes trajectory infeasible.');
        end
        t = [];
        return;
    end
    
    % If initial/final/average velocity happen to be the same, we're done
    if NumCompare(v1, v2, 'eq', margins) && NumCompare(v2, v_avg, 'eq', margins)
        if SHOW_COMMENTS
            display('BuildPLPTraj: Case 1.');
        end
        t = [s1; s2];
        return;
    end
    
    % If final velocity is less than average velocity, try final P curve -
    if NumCompare(v2, v_avg, 'lt', margins)
        
        a2 = a_min;
        
        % Calculate origin of final P curve
        del_t = -(v2 / a2);
        t_star = delta_t + del_t;
        x_star = motion(delta_x, v2, del_t, a2);
        
    % If it's greater, try + (if it's equal, we'll say it's + too)
    elseif NumCompare(v2, v_avg, 'ge', margins)
        
        a2 = a_max;
        
        % Calculate origin of final P curve
        x_star = delta_x - 0.5 * (v2^2) / a2;
        t_star = delta_t - v2 / a2;
        
    end

    % Decide whether initial acceleration should be + or -
    % If the initial velocity line intersects the acceleration curve, a1
    % should decelerate; otherwise accelerate
    a = a2;
    b = -2 * (a2 * t_star + v1);
    c = 2 * x_star + a2 * t_star^2;
    [tmp1 tmp2] = quadratic(a, b, c, margins);
    if ~isreal(tmp1) || ~isreal(tmp2)
        if NumCompare(a2, 0, 'lt', margins)
            a1 = a_min;
        else
            a1 = a_max;
        end
    else
        if NumCompare(a2, 0, 'lt', margins)
            a1 = a_max;
        else
            a1 = a_min;
        end
    end


    % Try to find a tangent line between the curves
    %[v1 a1 a2 x_star t_star]
    [ p1 p2 v ] = parabolasTangentLine( v1, a1, a2, x_star, t_star, margins );
    if ~isempty(p1) && ~isempty(p2)
        
        p1 = p1 + [x1 t1];
        p2 = p2 + [x1 t1];
        
        % Ensure that p1 and p2 are valid
        p1_x = getStateCoord(p1, 'x');
        p1_t = getStateCoord(p1, 't');
        p2_x = getStateCoord(p2, 'x');
        p2_t = getStateCoord(p2, 't');
        if NumCompare(p1_x, x1, 'ge', margins) && NumCompare(p1_t, t1, 'ge', margins)...
                && NumCompare(p2_x, x2, 'le', margins) && NumCompare(p2_t, t2, 'le', margins)
        
            % If the slope of the line is valid, we're done
            if inRange(v, v_feasible, margins)
                if SHOW_COMMENTS
                    display('BuildPLPTraj: Case 2a.');
                end
                                
                % If a valid line is found, we're done, build states and return
                t = [s1; [p1 v]; [p2 v]; s2];
                return;

            end
            
        end
        
    end

    % Try without linear segment
    %else
        a = a1 - a2;
        b = 2 * (a2 * delta_t - a1 * delta_t);
        c = 2 * delta_x - a2 * delta_t^2 - 2 * v1 * delta_t;
        [t1_plus t1_minus] = quadratic(a, b, c, margins);
        t1_tang = min([t1_plus t1_minus]);
        if ~isreal(t1_tang) || (t1_tang < 0)
            t1_tang = max([t1_plus t1_minus]);
        end

        % Validate time
        if isreal(t1_tang)...
                && NumCompare(t1_tang, 0, 'ge', margins)...
                && NumCompare(t1_tang, delta_t, 'le', margins)

            x1_tang = v1 * t1_tang + 0.5 * a1 * t1_tang^2;
            v_tang = v1 + a1 * t1_tang;
            %t_star = ((a2 - a1) * t1_tang - v1) / a2;
            v2_final = v_tang + a2 * (delta_t - t1_tang);
            if NumCompare(v2_final, v2, 'eq', margins)

                t = [s1; [x1+x1_tang t1+t1_tang v_tang]; s2];

                if SHOW_COMMENTS
                    display('BuildPLPTraj: Case 2b.');
                end
                return;

            end
        end
        
    %end
    
    % If nothing was found, try swapping final P curve sign
    if NumCompare(a2, a_max, 'eq', margins)
        a2 = a_min;

        % Calculate origin of final P curve
        del_t = -(v2 / a2);
        t_star = delta_t + del_t;
        x_star = motion(delta_x, v2, del_t, a2);
        
    else
        a2 = a_max;
        
        % Calculate origin of final P curve
        x_star = delta_x - 0.5 * (v2^2) / a2;
        t_star = delta_t - v2 / a2;
    end
    
    % Calculate origin of final P curve
    %x_star = delta_x - 0.5 * (v2^2) / a2;
    %t_star = delta_t - v2 / a2;
    
    % Decide whether initial acceleration should be + or -
    % If the initial velocity line intersects the acceleration curve, a1
    % should decelerate; otherwise accelerate
    a = a2;
    b = -2 * (a2 * t_star + v1);
    c = 2 * x_star + a2 * t_star^2;
    [tmp1 tmp2] = quadratic(a, b, c, margins);
    
    if ~isreal(tmp1) || ~isreal(tmp2)
        if NumCompare(a2, 0, 'lt', margins)
            a1 = a_min;
        else
            a1 = a_max;
        end
    else
        if NumCompare(a2, 0, 'lt', margins)
            a1 = a_max;
        else
            a1 = a_min;
        end
    end

    % Try to find a tangent line between the curves
    %[v1 a1 a2 x_star t_star]
    [ p1 p2 v ] = parabolasTangentLine( v1, a1, a2, x_star, t_star, margins );
    if ~isempty(p1) && ~isempty(p2)
        
        p1 = p1 + [x1 t1];
        p2 = p2 + [x1 t1];
        
        % Ensure that p1 and p2 are valid
        p1_x = getStateCoord(p1, 'x');
        p1_t = getStateCoord(p1, 't');
        p2_x = getStateCoord(p2, 'x');
        p2_t = getStateCoord(p2, 't');
        if NumCompare(p1_x, x1, 'ge', margins) && NumCompare(p1_t, t1, 'ge', margins)...
                && NumCompare(p2_x, x2, 'le', margins) && NumCompare(p2_t, t2, 'le', margins)

            % If the slope of the line is valid, we're done
            if inRange(v, v_feasible, margins)
                if SHOW_COMMENTS
                    display('BuildPLPTraj: Case 3a.');
                end

                % If a valid line is found, we're done, build states and return
                t = [s1; [p1 v]; [p2 v]; s2];
                return;

            end
            
        end
    
    end
    
    % Try without linear segment
    %else
        a = a1 - a2;
        b = 2 * delta_t * (a2 - a1);
        c = 2 * delta_x - a2 * delta_t^2 - 2 * v1 * delta_t;
        [t1_plus t1_minus] = quadratic(a, b, c, margins);
        t1_tang = min([t1_plus t1_minus]);
        if ~isreal(t1_tang) || (t1_tang < 0)
            t1_tang = max([t1_plus t1_minus]);
        end

        % Validate time
        if isreal(t1_tang)...
                && NumCompare(t1_tang, 0, 'ge', margins)...
                && NumCompare(t1_tang, delta_t, 'le', margins)

            x1_tang = v1 * t1_tang + 0.5 * a1 * t1_tang^2;
            v_tang = v1 + a1 * t1_tang;
            %t_star = ((a2 - a1) * t1_tang - v1) / a2;
            v2_final = v_tang + a2 * (delta_t - t1_tang);

            if NumCompare(v2_final, v2, 'eq', margins)

                t = [s1; [x1+x1_tang t1+t1_tang v_tang]; s2];

                if SHOW_COMMENTS
                    display('BuildPLPTraj: Case 3b.');
                end
                return;

            end
        end
    %end
    
    % If we get here, there's no valid trajectory
    if SHOW_COMMENTS
        display('BuildPLPTraj: No trajectory found.');
    end
    t = [];

end

