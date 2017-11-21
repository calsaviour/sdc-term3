% Initialize environment
setup;

% Contraints on the system
x_limit = 25;
t_limit = 25;

v_min = 0;
v_max = 10;

a_min = -10;
a_max = 8;

epsilon = 0.00000000015;

TEST_MARGINS = [.01 .01 .01 .01 .01 a_min a_max v_min v_max x_limit t_limit epsilon];

% Destination point (x, t)
p = [20 16];
%p = [4 2];
%p = [1 1];
%p = [12.375 2.25];

% Storage for set of states defining upper and lower bounding trajectories
s_upper = [];
s_lower = [];

% Attempt to find feasible bounding trajectories for the range of possible
% incoming velocities
for v0=v_min:v_max
%for v0 = 0
    
    s_i = [0 0 v0];

    s_upper = UpperBoundingStates(s_i, p, TEST_MARGINS);
    s_lower = LowerBoundingStates(s_i, p, TEST_MARGINS);

    s_s_upper = size(s_upper);
    s_s_lower = size(s_lower);
    
    cnt = 0;
    
    if s_s_upper(1, 1)>0
        cnt = cnt + 1;
        drawTrajectory(s_upper);
    end
    
    if s_s_lower(1, 1)>0
        cnt = cnt + 1;
        drawTrajectory(s_lower);
    end
    
    % It should never be the case that one bounding trajectory is found and
    % not the other, so, if that happens, do an info dump
    if cnt~=0 && cnt~=2
        v0
        s_upper
        s_lower
        display('MISMATCH');
    end
end