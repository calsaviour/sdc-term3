function [ B ] = Homotopic( T, O, P, margins )
%Homotopic Summary of this function goes here
%   Detailed explanation goes here

    SHOW_COMMENTS = 0;

    % ASSUME P IS SORTED BY T ASC
    
    % Bit vector signature
    B = [];
    
    % If trajectory is empty
    if isempty(T)
        if SHOW_COMMENTS
            display('T is empty.');
        end
        return;
    end
    
    % Check for collision
    c = NRS2CollisionCheckTrajectory(T, O, margins);
    if ~isempty(c)
        if SHOW_COMMENTS
            display('Collision detected. Trajectory: ');
            T
            display('Obstacles: ');
            c
        end
        return;
    end
    
    % Cycle through points, building bit field
    s_T = size(T);
    s_P = size(P);
    
    for i=1:s_P(1, 1)
        
        % Current point
        p = P(i, :);
        p_x = getStateCoord(p, 'x');
        p_t = getStateCoord(p, 't');
        
        %
        % Cycle through trajectory segments, finding the highest time for
        % this obstacle point
        %
        
        % Check this point across whole trajectory
        s1 = T(1, :);
        
        % Add the first point
        if pCompare([p_x p_t], [getStateCoord(s1, 'x') getStateCoord(s1, 't')], margins)
            
            % Get corner type
            type = getCornerType(p);

            % If this is a lower-right corner, traj is below
            if type==0
                B = cat(2, B, 0);

            % If this is an upper-left corner, traj is above
            else
                B = cat(2, B, 1);
            end
            
            continue;
        end
        
        for j=2:s_T(1, 1)
            
            s2 = T(j, :);
            
            % Skip duplicates in trajectory
            if sCompare(s1, s2, margins)
                s1 = s2;
                continue;
            end
            
            % Paramters for this segment
            x1 = getStateCoord(s1, 'x');
            t1 = getStateCoord(s1, 't');
            v1 = getStateCoord(s1, 'v');
            t2 = getStateCoord(s2, 't');
            v2 = getStateCoord(s2, 'v');
            
            % If the point is wholly above or below the time range of the trajectory, we're done
            %if (p_t <= t1) || (p_t > t2)
            if NumCompare(p_t, t1, 'le', margins) || NumCompare(p_t, t2, 'gt', margins)
                s1 = s2;
                continue;
            end
            
            %
            % At this point, p exists somewhere before or after the segment on the path
            %

            % Path coordinate of trajectory at p's time coordinate
            acc = (v2 - v1) / (t2 - t1);
            path_x = motion(x1, v1, p_t - t1, acc);
            
            % Compare this to p's path coordinate
            if NumCompare(path_x, p_x, 'eq', margins)
                
                % Get corner type
                type = getCornerType(p);

                % If this is a lower-right corner, traj is below
                if type==0
                    B = cat(2, B, 0);

                % If this is an upper-left corner, traj is above
                else
                    B = cat(2, B, 1);
                end
                
                break;
                
            %elseif path_x > p_x
            elseif NumCompare(path_x, p_x, 'gt', margins)
                B = cat(2, B, 0);
                break;
                
            else                
                B = cat(2, B, 1);
                s_B = size(B);
                break;
                
            end
            
        end
        
    end
    
    % In the absence of any viable points, we say by convention that the
    % trajectory belongs to the 0 class
    if isempty(B)
        B = 0;
    end

end

