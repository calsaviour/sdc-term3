% Initialize environment
setup;

% Contraints on the system
x_limit = 25;
t_limit = 25;

v_min = 0;
v_max = 10;
a_min = -4;
a_max = 4;

confidence_interval = 1;
half_confidence_interval = confidence_interval / 2;

a_min_range = [a_min-half_confidence_interval a_min+half_confidence_interval];
a_max_range = [a_max-half_confidence_interval a_max+half_confidence_interval];

epsilon = 0.00000000015;

% Destination point (x, t)
%p = [20 16];
p = [4 2];
%p = [4 4];
%p = [1 1];
%p = [12.375 2.25];

% Storage for set of states defining upper and lower bounding trajectories
s_upper = [];
s_lower = [];

% Start state
s_i = [0 0 0];

% Attempt to find feasible bounding trajectories for the range of possible controls
min_a_max = min(a_max_range);
max_a_max = max(a_max_range);
min_a_min = min(a_min_range);
max_a_min = max(a_min_range);

% Step size
inc = 0.1;

% Initial estimate
%TEST_MARGINS = [.01 .01 .01 .01 .01 mean(a_min_range) mean(a_max_range) v_min v_max x_limit t_limit epsilon];
TEST_MARGINS = [.01 .01 .01 .01 .01 min(a_min_range) max(a_max_range) v_min v_max x_limit t_limit epsilon];

s_upper = UpperBoundingStates(s_i, p, TEST_MARGINS);
s_lower = LowerBoundingStates(s_i, p, TEST_MARGINS);

if ~isempty(s_upper) && ~isempty(s_lower)

    for i=min_a_max:inc:max_a_max
    %for i=max_a_max:-inc:min_a_max
    %for i=max_a_max-inc:-inc:min_a_max+inc

        for j=max_a_min:-inc:min_a_min
        %for j=min_a_min:inc:max_a_min
        %for j=min_a_min+inc:inc:max_a_min-inc

            % New control range
            TEST_MARGINS = [.01 .01 .01 .01 .01 j i v_min v_max x_limit t_limit epsilon];

            % New control sequence
            upper_control_seq = Trajectory2ControlSequence(s_upper, TEST_MARGINS);
            lower_control_seq = Trajectory2ControlSequence(s_lower, TEST_MARGINS);

            % New trajectories
            s_upper_new = ControlSequence2Trajectory(upper_control_seq, getStateCoord(s_upper(1,:), 'v'), TEST_MARGINS);
            s_lower_new = ControlSequence2Trajectory(lower_control_seq, getStateCoord(s_lower(1,:), 'v'), TEST_MARGINS);
            drawTrajectory(s_upper_new, 'r');
            drawTrajectory(s_lower_new, 'r');
            break;
        end
        break;
    end
    
else
    
    display('No trajectories found.');

end