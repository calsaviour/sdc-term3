function [] = drawObstacleOverlay( num, traj, margins )
%drawObstacleOverlay Draw ground truth obstacle set over a sampled version
%   Draw ground truth obstacle set over a sampled version

    path = 'data/';
    ground_truth_name = 'ground_truth_obs_';
    sample_name = 'sample_obs_';
    offset_name = 'offset_';
    
    %ground_truth_path = [path, ground_truth_name, num2str(num)];
    ground_truth_path = [path, ground_truth_name, '0'];
    sample_path = [path, sample_name, num2str(num)];
    offset_path = [path, offset_name, num2str(num)];
    
    % load obstacle sets
    ground_truth = load( ground_truth_path );
    offset = load( offset_path );
    %sample = load( sample_path );
    
    %s_traj = size(traj);
    %for i=1:s_traj(1, 1)
    %    traj(i, :) = traj(i, :) + [offset 0];
    %end
    
    
    
    %drawObstacles( sample, margins );
    drawTrajectory( traj );
    drawObstaclesRelief( ground_truth, margins );
    plot(-offset(1,1), -offset(1,2), '-b+', 'LineWidth', 2, 'MarkerSize', 10);
    legend('Time-optimal trajectory', [210 195 200 75]);

end

