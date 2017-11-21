function [ sample_reachable ] = NextReachableRegion( s_i, t_measure, path_increment, margins )
%NextReachableRegion Compute reachable region in PV plane at time t_measure
%   Compute reachable region in PV plane at time t_measure

    % Determine reachable path bounds at t_measure
    p1 = [getStateCoord(s_i, 'x') getStateCoord(s_i, 't')];
    v_i = getStateCoord(s_i, 'v');

    V_i = [v_i v_i];
    [p_min p_max] = PathBoundsAtTime(s_i, t_measure, margins);

    % Generate sample points
    inc = (p_max - p_min) / path_increment;
    p_samples = p_min:inc:p_max;
    s_p_samples = path_increment + 1;

    % Calculate reachable sets at sample points
    % [p t v_min v_max]
    sample_reachable = [];
    for i=1:s_p_samples

        p2 = [p_samples(1, i) t_measure];
        [UB LB] = nextReachableSet2(p1, p2, V_i, margins );
        if isempty(UB) || isempty(LB)
            display('Encountered error in nextReachableSet');
            p1
            p2
            V_i
            break;
        end

        v_f_min = LB(end, end);
        v_f_max = UB(end, end);

        sample_reachable = cat(1, sample_reachable, [p2 v_f_min v_f_max]);

    end
    
end

