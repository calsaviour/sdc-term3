% Initialize environment
setup;

% Set of times for which to generate points
t_increment = 0.8;

% Sample increments
path_increment = 1000;

% Desired final velocity interval
V_f = [0 5];

% Determine reachable path bounds at t_measure
p1 = [0 0];
v_i = 0;

s_i = [p1 v_i];

figure;
hold on;
grid on;
daspect([1 1 1]);
xlim([0 20])
ylim([0 11]);
zlim([0 4]);
title({'Reachable Regions over Time','max velocity = 10 m/s'});
xlabel('Path (m)');
ylabel('Velocity (m/s)');
zlabel('Time (s)');
set(gca,'ZDir','reverse');
view(26, 34);

% PVT points
colors = [
    1 1 0; % yellow
    1 0 0; % red
    0 1 0; % green
    0 0 1; % blue
    ];
cnt = 1;
XYZ_upper = [];
XYZ_lower = [];
%for t=4:-t_increment:1
for t=t_increment:t_increment:3.2
    t
    % returns [p2 v_f_min v_f_max]
    sample_reachable = NextReachableRegion( s_i, t, path_increment, TEST_MARGINS );
    s_sample_reachable = size(sample_reachable);
    
    
    % Graph the region in the PV plane from which the goal is guaranteed to be reachable
    hull = [];
    for i=1:s_sample_reachable(1,1)
        tmp_lower = [sample_reachable(i,1) sample_reachable(i,2) sample_reachable(i,3)];
        tmp_upper = [sample_reachable(i,1) sample_reachable(i,2) sample_reachable(i,4)];
        XYZ_upper = cat(1, XYZ_upper, tmp_upper);
        XYZ_lower = cat(1, XYZ_lower, tmp_lower);
        tmp = cat(1, tmp_upper, tmp_lower);
        hull = cat(1, hull, tmp);
    end
    k = convhull(hull(:,1),hull(:,3));
    %plot3(hull(k,1), hull(k,3), hull(k,2));
    fill3(hull(k,1), hull(k,3), hull(k,2),colors(cnt,:));
    cnt = cnt + 1;
    %scatter3(hull(:,1), hull(:,3), hull(:,2));

end
line([9;9],[0;10],[2.4;2.4], 'Color', 'm')
