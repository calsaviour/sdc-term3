% Load environment
setup;

% Contraints on the system
x_limit = 6;
t_limit = 6;

v_min = 0;
v_max = 10;

a_min = -10;
a_max = 8;

epsilon = 0.00000000015;

TEST_MARGINS = [.01 .01 .01 .01 .01 a_min a_max v_min v_max x_limit t_limit epsilon];

% Define ground truth obstacle
width = 2.5;
height = 2.5;
center = [3 3];
O_gt = [center(1,1)-(width/2) center(1,1)+(width/2) center(1,2)-(height/2) center(1,2)+(height/2)];

% Generate obstacle samples
num_samples = 4;
std_deviation_path = 0.3;
std_deviation_time = 0.3;
face_alpha = 0.1;
confidence = 0.9995;

% Draw obstacles
O_samples = zeros(num_samples, 4);
r_path = std_deviation_path.*randn(num_samples,1);
r_time = std_deviation_time.*randn(num_samples,1);

path_sample_mu = mean(r_path);
path_sample_sigma = std(r_path);
time_sample_mu = mean(r_time);
time_sample_sigma = std(r_time);

% Compute confidence intervals
z_path = quant(confidence, center(1,1)+path_sample_mu, path_sample_sigma);
z_time = quant(confidence, center(1,2)+time_sample_mu, time_sample_sigma);
path_pad = z_path * path_sample_sigma / sqrt(num_samples);
time_pad = z_time * time_sample_sigma / sqrt(num_samples);

% Bounding obstacle
O_bounding = O_gt;
O_bounding = [O_gt(1,1)-path_pad O_gt(1,2)+path_pad O_gt(1,3)-time_pad O_gt(1,4)+time_pad];

% Draw bounding obstacle
drawObstacles(O_bounding, TEST_MARGINS);
hold on;

% Draw samples
C = [1 0 0];
for i=1:num_samples
    X = [O_gt(1,1)+r_path(i); O_gt(1,1)+r_path(i); O_gt(1,2)+r_path(i); O_gt(1,2)+r_path(i)];
    Y = [O_gt(1,3)+r_time(i); O_gt(1,4)+r_time(i); O_gt(1,4)+r_time(i); O_gt(1,3)+r_time(i)];
    patch( X, Y, C, 'FaceAlpha', face_alpha);
end
