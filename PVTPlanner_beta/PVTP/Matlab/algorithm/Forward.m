function [ G, Goal, intermediate_intervals ] = Forward( p0, V_i, V_f, O, margins )
%Forward Summary of this function goes here
%   Detailed explanation goes here

    RECORD_PROPAGATION = 0;

    % Count the number of intermediate intervals
    intermediate_intervals = 0;

    % This point will be designated as the coordinate of the goal
    x_limit = getMargin(margins, 'x_limit');
    t_limit = getMargin(margins, 't_limit');
    
    % Get obstacle corner points
    [UL LR] = segregatePoints(O);
    
    % Add point type designation to each:
    % UL = 1
    % LR = 0
    
    s_UL = size(UL);
    p_type = ones(s_UL(1, 1), 1);
    UL = cat(2, UL, p_type);
    
    s_LR = size(LR);
    p_type = zeros(s_UL(1, 1), 1);
    LR = cat(2, LR, p_type);
    
    % Combine into one set
    P = cat(1, UL, LR);
    
    % Add origin (a lower-right point by convention)
    P = cat(1, P, [p0 0]);
    
    % Remove points with invalid coordinates
    [badrows c] = find(P<0);
    P = P(setdiff(1:size(P,1),badrows),:);
    s_P = size(P);
        
    % Sort by time coordinate
    P_t = sortrows(P, [2 1]);
    
    % Storage for points with reachable intervals
    G(1, s_P(1, 1)) = struct('V', [], 'p', []);
    
    % Initialize with first point
    G(1).V = V_i;
    G(1).p = p0;
    
    % Storage for reachable intervals at goal
    Goal = struct('UB', [], 'LB', []);
    
    if RECORD_PROPAGATION
        h2 = figure('Position', [1 1 1024 480]);
        hold on;

        xlabel('Path (m)');
        ylabel('Time (s)');

        drawObstacles(O, margins);
        DrawReachableSets( G, margins );
        set(gca, 'Box', 'off');
        set(gcf,'PaperPositionMode','auto');

        %legend('Reachable sets', 'Propagation trajectories', 'Optimal trajectory');
        
        image_index = 0;
        print('-dpng',['data/images/forward_', num2str(image_index)]);
        image_index = image_index + 1;

        %hold off;
        %close(h2);
    end
    
    % Try to connect points
    for j=1:s_P(1, 1)
    %for j=1:20
    
        % Destination point
        p2 = P_t(j, :);
        
        %display(['Connecting point: [', num2str(p2(1, 1)), ' ', num2str(p2(1, 2)), ']...']);
        
        % Entry for this point in G
        G(j).p = p2;
        
        % Storage for reachable velocity intervals
        S_j = struct('Interval', G(j).V, 'B', [], 'traj1', [], 'traj2', []);

        % Only need to try connecting points before this in time
        for i=1:j-1
            
            % Source point
            p0 = P_t(i, :);
            
            % Initial velocity intervals at point i
            V_i = G(i).V;
            s_V_i = size(V_i);
            
            % For each disjoint interval in V_i
            cnt = 0;
            for k=1:s_V_i(1, 1)
                
                % Continuous velocity interval
                V_int = V_i(k, :);
                
                % Build set S for this connection
                [S_j cnt] = Propagate( p0, p2, V_int, S_j, O, P_t, margins );
                
                if (cnt > 0) && RECORD_PROPAGATION
                    %h2 = gcf;
                    %figure(h2);
                    %hold on;

                    %xlabel('Path (m)');
                    %ylabel('Time (s)');

                    %drawObstacles(O, margins);
                    %DrawReachableSets( G, margins );
                    %set(gca, 'Box', 'off');
                    
                    print('-dpng',['data/images/forward_', num2str(image_index)]);
                    image_index = image_index + 1;

                    %hold off;
                    %close(h2);
                end
                
            end
            
            intermediate_intervals = intermediate_intervals + cnt;

        end
        
        % Store information for this point
        V_i = Merge(S_j, margins);
        G(j).V = V_i;
        
        if ~isempty(V_i) && RECORD_PROPAGATION
            %h2 = gcf;
            %figure(h2);
            %hold on;

            %xlabel('Path (m)');
            %ylabel('Time (s)');

            %drawObstacles(O, margins);
            DrawReachableSets( G, margins );
            %set(gca, 'Box', 'off');

            print('-dpng',['data/images/forward_', num2str(image_index)]);
            image_index = image_index + 1;

            %hold off;
            %close(h2);
        end
        
        % Storage for reachable velocities at goal
        S_j = struct('UB', [], 'LB', [], 'B', []);

        % For each disjoint velocity interval, attempt to connect to goal
        s_V_i = size(V_i);
        
        cnt = 0;
        for k=1:s_V_i(1, 1)
            V_int = V_i(k, :);
            [S_j cnt] = PropagateGoal(G(j).p, V_int, V_f, S_j, O, P_t, margins);
            
            if cnt > 0 && RECORD_PROPAGATION
                %h2 = gcf;
                %figure(h2);
                %hold on;

                %xlabel('Path (m)');
                %ylabel('Time (s)');

                %drawObstacles(O, margins);
                DrawReachableSets( G, margins );
                %set(gca, 'Box', 'off');
                
                print('-dpng',['data/images/forward_', num2str(image_index)]);
                image_index = image_index + 1;

                %hold off;
                %close(h2);
            end
            
            intermediate_intervals = intermediate_intervals + cnt;
            
        end

        % Merge goal velocities, add to Goal intervals
        Goal = MergeGoal(S_j, Goal, margins);

    end
    
    % The first Goal element is a dummy, just remove it
    Goal(1) = [];

    if RECORD_PROPAGATION
        %h2 = gcf;
        %figure(h2);
        %hold on;

        %xlabel('Path (m)');
        %ylabel('Time (s)');

        %drawObstacles(O, margins);
        DrawReachableSets( G, margins );
        %set(gca, 'Box', 'off');

        print('-dpng',['data/images/forward_', num2str(image_index)]);
        image_index = image_index + 1;
        
        % Draw optimal trajectory
        t = GetOptimalTrajectory(G, Goal, O, margins);
        drawTrajectory(t, 'm');
        print('-dpng',['data/images/forward_', num2str(image_index)]);

        hold off;
        close(h2);
    end

end

