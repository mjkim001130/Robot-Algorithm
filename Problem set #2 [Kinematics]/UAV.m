clc;
clear;
close all;

hfig = figure(20);
hfig.Position(3:4) = [1120 840];
movegui(hfig);
grid on;
axis([-40, 300, -150, 150, -50, 500])
view(3);
title('UAV Simulation');
hold on;

[img1, ~, alpha1] = imread('UAV1.png'); 
[img2, ~, alpha2] = imread('UAV2.png'); 

% UAV's initial positions
x1 = 0; y1 = -50; z1 = 0;
x2 = 0; y2 = 50; z2 = 0;

imgSize = 20;

% Generate UAV1 coordinates
[X1, Y1, Z1] = getUAVCoordinates(x1, y1, z1, imgSize);

% Display UAV1 image
hUAV1 = surface('XData', X1, ...
                'YData', Y1, ...
                'ZData', Z1, ...
                'CData', flipud(img1), ...
                'FaceColor', 'texturemap', ...
                'EdgeColor', 'none', ...
                'AlphaData', flipud(alpha1), ... 
                'FaceAlpha', 'texturemap');

% Generate UAV2 coordinates
[X2, Y2, Z2] = getUAVCoordinates(x2, y2, z2, imgSize);

% Display UAV2 image
hUAV2 = surface('XData', X2, ...
                'YData', Y2, ...
                'ZData', Z2, ...
                'CData', flipud(img2), ...
                'FaceColor', 'texturemap', ...
                'EdgeColor', 'none', ...
                'AlphaData', flipud(alpha2), ... 
                'FaceAlpha', 'texturemap');

% Time settings
T_1 = 50;
T_2 = 80;
T_3 = 100;
T_4 = 130;
dt = 0.1;

% Initial parameters for UAV1
theta_roll1 = 0;
theta_pitch1 = 0;
theta_yaw1 = 0;

dx1 = 0;
dy1 = 0;
dz1 = 0;

% Initial parameters for UAV2
theta_roll2 = 0;
theta_pitch2 = 0;
theta_yaw2 = 0;

dx2 = 0;
dy2 = 0;
dz2 = 0;

% === t = 0 ~ T_1 (Ascending Phase) ===
for t = 0:dt:T_1
    % --- UAV1 Update ---
    % Update angles and positions
    theta_roll1 = (pi/6) * 0.5 * t;
    dx1 = 3 * t;
    dz1 = 2.5 * t; 
    
    % Compute transformation matrix
    H_r1 = H_matrix_roll(theta_roll1, dx1, dy1, dz1);
    H_t1 = eye(4);
    
    % Total transformation matrix
    H1 = H_t1 * H_r1;
    
    % Apply transformation to UAV1 coordinates
    [X1_new, Y1_new, Z1_new] = applyTransformation(H1, X1, Y1, Z1);
    
    % Update UAV1 position
    set(hUAV1, 'XData', X1_new, 'YData', Y1_new, 'ZData', Z1_new);
    
    % --- UAV2 Update ---
    % Update angles and positions
    theta_roll2 = -(pi/6) * 0.3 * t; 
    dx2 = 3 * t; 
    dz2 = 2.5 * t; 
    
    % Compute transformation matrix
    H_r2 = H_matrix_roll(theta_roll2, dx2, dy2, dz2);
    H_t2 = eye(4);
    
    % Total transformation matrix
    H2 = H_t2 * H_r2;
    
    % Apply transformation to UAV2 coordinates
    [X2_new, Y2_new, Z2_new] = applyTransformation(H2, X2, Y2, Z2);
    
    % Update UAV2 position
    set(hUAV2, 'XData', X2_new, 'YData', Y2_new, 'ZData', Z2_new);
    
    drawnow;
    pause(0.01);
end

% Initialize values to prevent abrupt changes
dx1_init = dx1;
dx2_init = dx2;
dz1_init = dz1;
dz2_init = dz2;

% === t = T_1 ~ T_2 (Transition Phase) ===
for t = T_1:dt:T_2
    % Normalize time step
    t_step = (t - T_1) / (T_2 - T_1);

    % Update pitch angles
    theta_pitch1 = pi * t_step;
    theta_pitch2 = pi * t_step;

    % Decrease forward speed smoothly
    dx1 = dx1_init * (1 - t_step);
    dx2 = dx2_init * (1 - t_step);

    % Continue ascending
    dz1 = dz1_init + 3 * (t - T_1);
    dz2 = dz2_init + 3 * (t - T_1);

    % --- UAV1 Update ---
    H_p1 = H_matrix_pitch(theta_pitch1, dx1, dy1, dz1);
    H_t1 = eye(4);
    H1 = H_t1 * H_p1;
    [X1_new, Y1_new, Z1_new] = applyTransformation(H1, X1, Y1, Z1);
    set(hUAV1, 'XData', X1_new, 'YData', Y1_new, 'ZData', Z1_new);

    % --- UAV2 Update ---
    H_p2 = H_matrix_pitch(theta_pitch2, dx2, dy2, dz2);
    H_t2 = eye(4);
    H2 = H_t2 * H_p2;
    [X2_new, Y2_new, Z2_new] = applyTransformation(H2, X2, Y2, Z2);
    set(hUAV2, 'XData', X2_new, 'YData', Y2_new, 'ZData', Z2_new);

    drawnow;
    pause(0.01);
end

% Set initial values for cruising phase
dx1_cruise = 5; % Adjust cruising speed as needed
dx2_cruise = 5;

% Set initial positions at T_2
dx1_init = dx1;
dx2_init = dx2;
dz1_init = dz1;
dz2_init = dz2;

% === t = T_2 ~ T_3 (Cruising Phase) ===
for t = T_2:dt:T_3
    % --- UAV1 Update ---
    theta_pitch1 = pi/2; 
    dx1 = dx1 + dx1_cruise * dt; 
    dz1 = dz1_init; 

    H_p1 = H_matrix_pitch(theta_pitch1, dx1, dy1, dz1);
    H_t1 = eye(4);
    H1 = H_t1 * H_p1;
    [X1_new, Y1_new, Z1_new] = applyTransformation(H1, X1, Y1, Z1);
    set(hUAV1, 'XData', X1_new, 'YData', Y1_new, 'ZData', Z1_new);

    % --- UAV2 Update ---
    theta_pitch2 = pi/2; 
    dx2 = dx2 + dx2_cruise * dt; 
    dz2 = dz2_init; 

    H_p2 = H_matrix_pitch(theta_pitch2, dx2, dy2, dz2);
    H_t2 = eye(4);
    H2 = H_t2 * H_p2;
    [X2_new, Y2_new, Z2_new] = applyTransformation(H2, X2, Y2, Z2);
    set(hUAV2, 'XData', X2_new, 'YData', Y2_new, 'ZData', Z2_new);

    drawnow;
    pause(0.01);
end

% Landing parameters
dz1_landing_start = dz1; % Altitude at the start of landing
dz2_landing_start = dz2;
landing_duration = T_4 - T_3;

% === t = T_3 ~ T_4 (Landing Phase) ===
for t = T_3:dt:T_4
    % Normalize time for smooth 
    t_step = (t - T_3) / landing_duration;

    % --- UAV1 Update ---
    theta_pitch1 = pi/2 * (1 - t_step); % Gradually decrease pitch angle
    dx1 = dx1 + dx1_cruise * dt; % Continue moving forward
    dz1 = dz1_landing_start * (1 - t_step); % Gradually decrease altitude

    H_p1 = H_matrix_pitch(theta_pitch1, dx1, dy1, dz1);
    H_t1 = eye(4);
    H1 = H_t1 * H_p1;
    [X1_new, Y1_new, Z1_new] = applyTransformation(H1, X1, Y1, Z1);
    set(hUAV1, 'XData', X1_new, 'YData', Y1_new, 'ZData', Z1_new);

    % --- UAV2 Update ---
    theta_pitch2 = pi/2 * (1 - t_step); % Gradually decrease pitch angle
    dx2 = dx2 + dx2_cruise * dt; % Continue moving forward
    dz2 = dz2_landing_start * (1 - t_step); % Gradually decrease altitude

    H_p2 = H_matrix_pitch(theta_pitch2, dx2, dy2, dz2);
    H_t2 = eye(4);
    H2 = H_t2 * H_p2;
    [X2_new, Y2_new, Z2_new] = applyTransformation(H2, X2, Y2, Z2);
    set(hUAV2, 'XData', X2_new, 'YData', Y2_new, 'ZData', Z2_new);

    drawnow;
    pause(0.01);
end

%close(video);
hold off;

% Function to generate UAV coordinates
function [X, Y, Z] = getUAVCoordinates(x, y, z, imgSize)
    X = [x - imgSize/2, x + imgSize/2; x - imgSize/2, x + imgSize/2];
    Y = [y - imgSize/2, y - imgSize/2; y + imgSize/2, y + imgSize/2];
    Z = [z, z; z, z];
end

% Function to apply transformation matrix to coordinates
function [X_new, Y_new, Z_new] = applyTransformation(H, X, Y, Z)
    numPoints = numel(X);
    coords = [X(:)'; Y(:)'; Z(:)'; ones(1, numPoints)];
    coords_new = H * coords;
    X_new = reshape(coords_new(1, :), size(X));
    Y_new = reshape(coords_new(2, :), size(Y));
    Z_new = reshape(coords_new(3, :), size(Z));
end

function H = H_matrix_roll(theta, dx, dy, dz)
    H = [1, 0,           0,          dx;
         0, cos(theta), -sin(theta), dy;
         0, sin(theta),  cos(theta), dz;
         0, 0,           0,          1];
end

function H = H_matrix_pitch(theta, dx, dy, dz)
    H = [ cos(theta), 0, sin(theta), dx;
          0,          1, 0,          dy;
         -sin(theta), 0, cos(theta), dz;
          0,          0, 0,          1];
end
