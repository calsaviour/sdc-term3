function [ V_i ] = FindInitialVelocityRange( p0, p2, margins )
%FindInitialVelocityRange Calculate the set of feasible initial velocities
%   Calculate the set of feasible initial velocities

    V_i = [];

    SHOW_COMMENTS = 0;

    % The bounds
    v_min = getMargin(margins, 'vel_min');
    v_max = getMargin(margins, 'vel_max');
    a_min = getMargin(margins, 'acc_min');
    a_max = getMargin(margins, 'acc_max');
    
    x0 = getStateCoord(p0, 'x');
    t0 = getStateCoord(p0, 't');
    x2 = getStateCoord(p2, 'x');
    t2 = getStateCoord(p2, 't');
    
    delta_x = x2 - x0;
    delta_t = t2 - t0;
    
    v_avg = delta_x / delta_t;
    
    % Quick checks: if average velocity required to reach endpoint is outside
    if NumCompare(v_avg, v_max, 'gt', margins) || NumCompare(v_avg, v_min, 'lt', margins)
        V_i = [];
        return;
    end
    
    %
    % First determine the maximum possible incoming velocity
    %
    
    % Find path coordinate where parabola zeroes assuming v0 = v_max
    v0 = v_max;
    a1 = a_min;
    
    x = -0.5 * (v0^2) / a1;
    a = a1;
    b = 2 * v0;
    c = -2 * x;
    [t1 t1_minus] = quadratic(a, b, c, margins);
    
    % If the zero point is beyond the end point, adjust the parabola
    % backward by decreasing the initial velocity
    if NumCompare(x, delta_x, 'gt', margins) % || NumCompare(t1, delta_t, 'gt', margins)
        
        if SHOW_COMMENTS
            display('Max, case 1.');
        end
        
        % The initial velocity that reaches the endpoint via constant deceleration
        %v0 = (delta_x - 0.5 * a1 * delta_t^2) / delta_t;
        v0 = (delta_x / delta_t) - 0.5 * a1 * delta_t;
        
        % Make sure final velocity is valid
        v_f = v0 + a1 * delta_t;
        if NumCompare(v_f, v_min, 'lt', margins)
            
            % The initial velocity that mins at x2
            v0 = sqrt( -2 * delta_x * a1 );

            % Make sure this satisfies the time constraint; it will always
            % satisfy velocity constraints
            a = a1;
            b = 2 * v0;
            c = -2 * delta_x;
            [t1 t1_minus] = quadratic(a, b, c, margins);

            % If this test fails, final velocity is > v_min
            if isreal(t1) && (NumCompare(t1, delta_t, 'gt', margins) || NumCompare(t1, 0, 'lt', margins))

                % Verify v0
                if NumCompare(v0, v_max, 'gt', margins)

                    % If we get here, there is no dynamically feasible initial velocity
                    % that can reach the endpoint via constant deceleration; try introducing
                    % an initial linear segment
                    %v0 = v_max;
                    a = 1;
                    b = -2 * delta_t;
                    c = delta_t^2 + 2 * v_max * delta_t - 2 * delta_x / a1;
                    [t1 t_minus] = quadratic( a, b, c, margins );

                    % If t1 is not okay, fail out
                    if NumCompare(t1, 0, 'lt', margins) || NumCompare(t1, delta_t, 'gt', margins)
                        V_i = [];
                        return;
                    end

                    v0 = v_max;
                end

            end
            
        end
        
        %{
        % The initial velocity that mins at x2
        v0 = sqrt( -2 * delta_x * a1 );
        
        % Make sure this satisfies the time constraint; it will always
        % satisfy velocity constraints
        a = a1;
        b = 2 * v0;
        c = -2 * delta_x;
        [t1 t1_minus] = quadratic(a, b, c, margins);
        
        % If this test fails, final velocity is > v_min
        if isreal(t1) && (NumCompare(t1, delta_t, 'gt', margins) || NumCompare(t1, 0, 'lt', margins))

            % The initial velocity that reaches the endpoint via constant deceleration
            v0 = (delta_x - 0.5 * a1 * delta_t^2) / delta_t;
            
            % Make sure final velocity is valid
            v_f = v0 + a1 * delta_t;
            if NumCompare(v_f, v_min, 'lt', margins)
                V_i = [];
                return;
            end
        
            % Verify v0
            if NumCompare(v0, v_max, 'gt', margins)

                % If we get here, there is no dynamically feasible initial velocity
                % that can reach the endpoint via constant deceleration; try introducing
                % an initial linear segment
                %v0 = v_max;
                a = a1;
                b = -2 * a1 * delta_t;
                c = a1 * delta_t^2 + 2 * v_max * delta_t - 2 * delta_x;
                [t1 t_minus] = quadratic( a, b, c, margins );

                % If t1 is not okay, fail out
                if NumCompare(t1, 0, 'lt', margins) || NumCompare(t1, delta_t, 'gt', margins)
                    V_i = [];
                    return;
                end

                v0 = v_max;
            end

        end
        %}
        % If we get to this point, we're good, and maximum incoming
        % velocity is v0. Use a 0 placeholder for min for now
        V_i = [0 v0];
        
    else
        
        if SHOW_COMMENTS
            display('Max, case 2.');
        end

        % Use the acceleration that will yield a shorter line segment
        if NumCompare(v0, v_avg, 'gt', margins)
            a1 = a_min;
        else
            a1 = a_max;
        end
    
        % If it zeros before the end point, ensure that velocity constraints
        % are respected by finding the tangent line connecting the parabola
        % with the endpoint and verifying its slope
        a = a1;
        b = -2 * a1 * delta_t;
        c = 2 * delta_x - 2 * v0 * delta_t;
        [t1 t1_minus] = quadratic(a, b, c, margins);

        % Quick sanity check
        if NumCompare(t1, delta_t, 'gt', margins) || NumCompare(t1, 0, 'lt', margins)
            V_i = [];
            return;
        end

        % The tangent point
        v1 = v0 + a1 * t1;
        x1 = v1 * t1 + delta_x - v1 * delta_t;

        % Quick sanity check
        if NumCompare(x1, delta_x, 'gt', margins) || NumCompare(x1, 0, 'lt', margins)
            V_i = [];
            return;
        end

        % Now make sure that the velocity needed to reach the end is
        % within constraints
        v = (delta_x - x1) / (delta_t - t1);
        if NumCompare(v, v_max, 'gt', margins) || NumCompare(v, v_min, 'lt', margins)
            V_i = [];
            return;
        end

        % By this point, everything checks out; use a zero placeholder for
        % minimum incoming velocity for now
        V_i = [0 v0];
        
    end
    
    %
    % Now minimum possible incoming velocity
    %
    
    % Will max acc with v0 = v_min reach the time limit at x2?
    a1 = a_max;
    v0 = v_min;
    a = a1;
    b = 2 * v0;
    c = -2 * delta_x;
    [t1 t1_minus] = quadratic(a, b, c, margins);
    
    if NumCompare(t1, delta_t, 'gt', margins)
        
        if SHOW_COMMENTS
            display('Min, case 1.');
        end
        
        % If the arrival time is greater than the given time, adjust v0
        % such that t1 = t2
        v0 = (delta_x - 0.5 * a1 * delta_t^2) / delta_t;
        
        % Quick check
        if NumCompare(v0, v_max, 'gt', margins) || NumCompare(v0, v_min, 'lt', margins)
            V_i = [];
            if SHOW_COMMENTS
                display('Min, case 1: velocity bounds violated (1).');
            end
            return;
        end
        
        % Quick check of arrival velocity
        v2 = v0 + a1 * delta_t;
        if NumCompare(v2, v_max, 'gt', margins)
            
            % If it's greater drop a linear segment at v_max from
            % destination, find the parabola tangent to it
            x_int = delta_x - v_max * delta_t;
            a = 1;
            b = -2 * x_int;
            c = x_int^2 + (2 * x_int * v_max^2) / a1;
            [x1 x1_minus] = quadratic(a, b, c, margins);
            t1 = (x1 - x_int) / v_max;
            v0 = v_max - a1 * t1;
            
            if NumCompare(v0, v_min, 'lt', margins) || NumCompare(v0, v_max, 'gt', margins)
                V_i = [];
                if SHOW_COMMENTS
                    display('Min, case 1: velocity bounds violated (2).');
                end
                return;
            end
            
        end          
        
        % If we make it to here, everything checks out
        V_i = [v0 max(V_i)];
        
    else
        
        if SHOW_COMMENTS
            display('Min, case 2.');
        end
        
        % If time limit is not initially violated, verify that velocity 
        % constraints are respected by finding the tangent line connecting
        % the parabola with the endpoint and verifying its slope
        a = a1;
        b = -2 * a1 * delta_t;
        c = 2 * delta_x - 2 * v0 * delta_t;
        [t1_plus t1] = quadratic(a, b, c, margins);

        % Quick sanity check
        if NumCompare(t1, delta_t, 'gt', margins) || NumCompare(t1, 0, 'lt', margins)
            V_i = [];
            return;
        end

        % The tangent point
        v1 = v0 + a1 * t1;
        x1 = v1 * t1 + delta_x - v1 * delta_t;

        % Quick sanity check
        if NumCompare(x1, delta_x, 'gt', margins) || NumCompare(x1, 0, 'lt', margins)
            V_i = [];
            return;
        end

        % Now make sure that the average velocity needed to reach the end is
        % within constraints
        numerator = delta_x - x1;
        denominator = delta_t - t1;
        if NumCompare(numerator, 0, 'eq', margins)
            v = 0;
        else
            v = numerator / denominator;
        end
        if NumCompare(v, v_max, 'gt', margins) || NumCompare(v, v_min, 'lt', margins)
            V_i = [];
            return;
        end

        % By this point, everything checks out; use a zero placeholder for
        % minimum incoming velocity for now
        V_i = [v0 max(V_i)];
        
    end

end

