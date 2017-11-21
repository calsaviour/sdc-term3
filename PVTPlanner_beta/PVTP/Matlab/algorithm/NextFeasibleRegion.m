function [ valid_sample_points sample_reachable ] = NextFeasibleRegion( s_i, t_measure, path_increment, V_f, O, margins )
%NextFeasibleRegion Compute feasible region in PV plane at time t_measure
%   Compute feasible region in PV plane at time t_measure

    sample_reachable = NextReachableRegion( s_i, t_measure, path_increment, margins );
    s_sample_reachable = size(sample_reachable);
    
    s_Obs = size(O);
    
    % For each obstacle, attempt a goal connection from a sample point, keep only 
    % those points for which the goal is reachable in the presence of any obstacle
    valid_sample_points = [];
    for i=1:s_sample_reachable(1, 1)

        p = sample_reachable(i, 1:2);
        V_i = sample_reachable(i, 3:4);

        x_offset = -getStateCoord(p, 'x');
        t_offset = -getStateCoord(p, 't');

        safe = 1;
        for j=1:s_Obs(1, 1)

            % Translate system s.t. p is the origin
            O_loop = translateObstacles(x_offset, t_offset, O);

            % Run propagation
            [G Goal] = Forward([0 0], V_i, V_f, O_loop, margins);
            s_Goal = size(Goal);

            % If this configuration has no goal reachable trajectory, discard sample point
            if s_Goal(1, 2) == 0
                safe = 0;
                break;
            end

        end

        % If this sample point could reach the goal regardless of where the obstacle ended up, it's safe
        if safe == 1
            valid_sample_points = cat(1, valid_sample_points, sample_reachable(i, :));
        end

    end

end

