% Initialize environment
setup;

% Time of state measurement
t_measure = 1;

% Sample increments
path_increment = 10;

% Desired final velocity interval
V_f = [0 5];

% Determine reachable path bounds at t_measure
p1 = [0 0];
v_i = 0;

s_i = [p1 v_i];

% Uncertain obstacles: These obstacles represent possible paths for a given obstacle
%{
% forward section
UNCERTAIN_OBSTACLES = [
    1   3   1   3;
    3   5   3   5;
    5   7   1   3;
    ];
%}
%{
% divided region, forward
UNCERTAIN_OBSTACLES = [
    0.5   2   0   2;
    4.5   6   0   2;
    ];
%}
%{
% divided region backward
UNCERTAIN_OBSTACLES = [
   -0.4014    1.5986   -0.3539    0.6461;
    0.8374    2.8374    4.1443    5.1443;
    4.6071    6.6071    3.1517    4.1517;
    ];
%}
%{
% divided region backward
UNCERTAIN_OBSTACLES = [
   -0.3090    1.6910    2.8986    3.8986;
    0.8331    2.8331    0.1828    1.1828;
    1.3475    3.3475    3.1061    4.1061;
    ];
%}
%{
% full region
UNCERTAIN_OBSTACLES = [
    4.7270    6.7270    0.2493    1.2493;
    4.7228    6.7228    2.7980    3.7980;
    4.0571    6.0571    2.0930    3.0930;
    ];
%}
%UNCERTAIN_OBSTACLES = GenerateRandomObstacles( 3, TEST_MARGINS );

% planning test
UNCERTAIN_OBSTACLES = [
    1.1241    2.6241    4.1877    4.6877;
    0.4210    1.9210    2.0534    2.5534;
    2.2813    3.7813    1.0874    1.5874;
    ];

O = translateObstacles( 0, t_measure, UNCERTAIN_OBSTACLES );
figure;
drawObstacles(O, TEST_MARGINS);
hold on;
x_limit = getMargin(TEST_MARGINS, 'x_limit');
plot([0; x_limit], [t_measure; t_measure], '--b');
legend('Sensing time', 'Location', 'SouthEast');

[valid_sample_points sample_reachable] = NextFeasibleRegion(s_i, t_measure, path_increment, V_f, O, TEST_MARGINS);
s_valid_sample_points = size(valid_sample_points);

% Graph the region in the PV plane from which the goal is guaranteed to be reachable
if s_valid_sample_points(1, 1) == 0
    return;
end
    
X_reachable = cat(1, sample_reachable(:, 1), sample_reachable(:, 1));
Y_reachable = cat(1, sample_reachable(:, 3), sample_reachable(:, 4));
X_feasible = cat(1, valid_sample_points(:, 1), valid_sample_points(:, 1));
Y_feasible = cat(1, valid_sample_points(:, 3), valid_sample_points(:, 4));

k_reachable = convhull(X_reachable, Y_reachable);
k_feasible = convhull(X_feasible, Y_feasible);

P_feasible = [X_feasible(k_feasible) Y_feasible(k_feasible)];

% find centroid
[IDX C] = kmeans(P_feasible, 1);

figure;
plot(X_reachable(k_reachable), Y_reachable(k_reachable), 'r-');
hold on;
plot(P_feasible(:,1), P_feasible(:,2), 'b-');
plot(C(1,1), C(1,2), '-mo', 'LineWidth', 2, 'MarkerSize', 4);

legend('Reachable region', 'Feasible region', 'Centroid', 'Location', 'NorthWest');

% target state
s_2 = [C(1,1) t_measure C(1,2)];
