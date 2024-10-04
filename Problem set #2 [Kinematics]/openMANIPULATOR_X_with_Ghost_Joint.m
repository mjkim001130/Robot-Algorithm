clc; clear; close all;

% DH parameters 
DH_params = [
   %a_i    alpha_i  d_i      theta_i 
    0,     pi/2,    0.77,     0;    % Link 1
    1.28, 0,      0,         0; % ghost joint
    0.5,   0,      0,         0;    % Link 2 원래 0.24이지만, 잘 안보여서 0.8로 수정
    1.24,  0,      0,         0; % Link 3 
    1.26,  0,      0,         0;    % Link 4
];


% Create figure and axis
figure('Position', [100, 100, 1200, 800]);
axis([-10, 10, -10, 10, -2, 5]);
grid on;
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('openMANIPULATOR-X with ghost joint');
hold on;

% Set base position
plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Number of links
num_links = size(DH_params, 1);

% Initialize handle arrays for plotting
hLinks = gobjects(num_links, 1);
hJoints = gobjects(num_links+1, 1); 

% Initialize plot objects for links and joints
for i = 1:num_links
    hLinks(i) = plot3([0, 0], [0, 0], [0, 0], 'LineWidth', 3);
    hJoints(i+1) = plot3(0, 0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
end
hJoints(1) = plot3(0, 0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% Initialize end effector visualization
hEndEffector = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Red dot for end effector

% Colors for each link
colors = lines(num_links);

for t = 0:0.1:30
    % Define joint angles as functions of time
    theta = [
        pi/3 * t;              % Joint 1 (Revolute)
        pi/2;                      % ghost joint
        pi/6 * t;      % Joint 2 (Revolute)
        (pi/9 - pi/2) * t;      % Joint 3 (Revolute)
        pi/12 * t;              % Joint 4 (Revolute)
    ];

    % Initialize the transformation matrix
    T = eye(4);
    positions = zeros(num_links+1, 3); % Store positions of joints
    positions(1, :) = [0, 0, 0]; % base position

    % Forward kinematics for each link
    for i = 1:num_links
        a = DH_params(i, 1);
        alpha = DH_params(i, 2);
        d = DH_params(i, 3);
        theta_i = theta(i) + DH_params(i, 4); % Add theta offset from DH params

        % Compute transformation matrix using DH convention
        A = DH_Convention(theta_i, d, a, alpha);
        T = T * A; % Update overall transformation matrix
        
        % Store the position of the current joint
        positions(i+1, :) = T(1:3, 4)'; % Extract position vector
    end

    % Update the plots for links and joints
    for i = 1:num_links
        set(hLinks(i), 'XData', positions(i:i+1, 1), 'YData', positions(i:i+1, 2), 'ZData', positions(i:i+1, 3), 'Color', colors(i, :));
        set(hJoints(i+1), 'XData', positions(i+1, 1), 'YData', positions(i+1, 2), 'ZData', positions(i+1, 3));
    end

    % Update the position of the end effector (last joint's position)
    end_effector_pos = positions(end, :); % The end-effector is at the last joint's position
    set(hEndEffector, 'XData', end_effector_pos(1), 'YData', end_effector_pos(2), 'ZData', end_effector_pos(3));
    
    drawnow;
    pause(0.1);
    
end