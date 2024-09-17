clc; clear; close all;

%% Set Environment

% Start and goal

start = [1,5];
goal = [18, 5];

figure('Position', [100, 100, 800, 600]);
hold on;
grid on;
xlabel('X');
ylabel('Y');
title('Bug 1');

plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start point
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);   % Goal point

% obstacle
obstacle1 = [3, 8; 5, 8; 8, 5; 5, 2; 3, 2; 5,5]; 
theta = linspace(0, 2*pi, 100);
obstacle2 = [12 + 2*cos(theta)', 5 + 2*sin(theta)'];

obstacles = {obstacle1, obstacle2};


for i = 1:length(obstacles)
    obs = obstacles{i};
    fill(obs(:,1), obs(:,2), [0.5 0.5 0.5]); 
end

xlim([-1, 20]);
ylim([-1, 12]);

%% Initial the bug 

current_pos = start;
path = current_pos; 

bug_plot = plot(current_pos(1), current_pos(2), 'bo', 'MarkerSize', 5, 'LineWidth', 2);

max_step = 100000;
step_size = 0.05;

%% Main

for step = 1 : max_step
    % Find the goal
    if norm(goal - current_pos) < step_size
        disp('Find the Goal!!!!!!!!!!!!!!!!!');
        break;
    end

    % Motion-to-goal
    direction = (goal - start);
    direction = direction / norm(direction); % unit vector
    next_pos = current_pos + direction * step_size;

    collision = false; % Trigger
    for i = 1:length(obstacles)
        obs = obstacles{i};
        num_edge = size(obs, 1); % 장애물의 꼭짓점 개수를 저장함
        for j = 1:num_edge % 장애물들의 각 변에 대해 탐색
            % obs(j, :) -> 현재 꼭짓점의 좌표 [x,y]
            % 다각형의 마지막 변이 첫번째와 연결되게
            x_edge = [obs(j, 1), obs(mod(j, num_edge)+1, 1)]; 
            y_edge = [obs(j, 2), obs(mod(j, num_edge)+1, 2)]; 

            % line tracing
            x_line = [current_pos(1), next_pos(1)];
            y_line = [current_pos(2), next_pos(2)];

            % check collision
            [xi, yi] = polyxpoly(x_line, y_line, x_edge, y_edge);

            if ~isempty(xi)
                collision = true;
                collided_obs = obs;
                q_H = [xi(1), yi(1)];
                edge_start_idx = j;
                break;
            end
        end
        if collision
            break;
        end
    end
    
    if ~collision
        current_pos = next_pos;
        path = [path; current_pos]; % add current_pos to path

        % update plot
        set(bug_plot, 'XData', current_pos(1), 'YData', current_pos(2));

        if size(path, 1) > 1
            plot(path(end-1:end,1), path(end-1:end,2), 'r', 'LineWidth',2);
        end
        drawnow;
        pause(0.001);
    else
        %% Boundary following
        disp('Collosion! Begining boundary following');
        % plot hit point
        plot(q_H(1), q_H(2), 'ko', 'MarkerSize', 8, 'LineWidth',2);

        % start boundary following
        [current_pos, boundary_path, q_L] = boundary_following(q_H, goal, collided_obs, step_size, bug_plot, edge_start_idx);
        % Add boundary_path to path
        path = [path; boundary_path];

        plot(q_L(1), q_L(2), 'mo', 'MarkerSize', 8, 'LineWidth', 2);

        current_pos = q_L;
    end
end

%% Final Path Plot

plot(path(:,1), path(:,2), 'r', 'LineWidth',2);

hold off;

%% Function Boundary following

function [current_pos, boundary_path, q_L] = boundary_following(current_pos, goal, obstacle, step_size, bug_plot, edge_start_idx)
    boundary_path = [];
    distance_to_goal = [];

    % Get obstacle vertex
    obs_vertex = obstacle; % 꼭짓점 좌표 저장
    num_vertex = size(obs_vertex, 1); % 개수 저장

    start_point = current_pos; % hit point = q_H
    finish_loop = false; % Trigger for boundary following finished

    current_edge_start_idx = edge_start_idx;
    current_edge_end_idx = mod(current_edge_start_idx, num_vertex) + 1;

    while ~finish_loop
        start_vertex = obs_vertex(current_edge_start_idx, :);
        end_vertex = obs_vertex(current_edge_end_idx, :);

        % compute vector of edge
        edge_vector = end_vertex - start_vertex; % start to end
        edge_length = norm(edge_vector);
        edge_direction = edge_vector / edge_length; % unit vector

        remaining_vector = end_vertex - current_pos;
        remaining_length = norm(remaining_vector);
        remaining_direction = remaining_vector / remaining_length; % unit vector

        % Num of step to following edge
        num_steps = ceil(remaining_length / step_size); % ceil 함수는 소수점 이하 올림

        % following the edge
        for s = 1:num_steps
            current_pos = current_pos + remaining_direction * step_size; % move toward direction
            boundary_path = [boundary_path; current_pos]; % Add current_pos to path matrix
            distance_to_goal = [distance_to_goal; norm(goal - current_pos)];

            % Update bug
            set(bug_plot, 'XData', current_pos(1), 'YData', current_pos(2));
            if size(boundary_path, 1) > 1
                plot(boundary_path(end-1:end,1), boundary_path(end-1:end,2), 'r', 'LineWidth',2);
            end
            drawnow;
            pause(0.01);

            % if current_pos == hit point
            if norm(current_pos - start_point) < step_size && size(boundary_path,1) > num_steps
                finish_loop = true;
                break;
            end
        end

        % go to next edge
        current_edge_start_idx = current_edge_end_idx;
        current_edge_end_idx = mod(current_edge_start_idx, num_vertex) + 1;

        % 현재 위치를 다음 엣지의 시작으로 reset
        current_pos = obs_vertex(current_edge_start_idx, :);
    end

    % Find the Leave point q_L
    % 경계를 따라 이동하면서 기록된 지점중 가장 가까운점 찾음
    [~, min_idx] = min(distance_to_goal); 
    q_L = boundary_path(min_idx, :);

    % hit point에 방문하면, leave point로 이동
    current_pos = start_point;

    path_to_q_L = [];
    is_q_L = false;

    current_edge_start_idx = edge_start_idx; 
    current_edge_end_idx = mod(current_edge_start_idx, num_vertex) + 1;

    while ~is_q_L
        % 위에서 이동한것과 동일함
        start_vertex = obs_vertex(current_edge_start_idx, :);
        end_vertex = obs_vertex(current_edge_end_idx, :);

        edge_vector = end_vertex - start_vertex;
        edge_length = norm(edge_vector);
        edge_direction = edge_vector / edge_length;

        remaining_vector = end_vertex - current_pos;
        remaining_length = norm(remaining_vector);
        remaining_direction = remaining_vector / remaining_length;

        num_steps = ceil(remaining_length / step_size);

        for s = 1:num_steps
            current_pos = current_pos + remaining_direction * step_size;
            path_to_q_L = [path_to_q_L; current_pos];
            
            % Update bug's position on the plot
            set(bug_plot, 'XData', current_pos(1), 'YData', current_pos(2));
            if size(path_to_q_L,1) > 1
                plot(path_to_q_L(end-1:end,1), path_to_q_L(end-1:end,2), 'r', 'LineWidth', 2);
            end
            drawnow;
            pause(0.01);

            % Check is q_L?
            if norm(current_pos - q_L) < step_size
                is_q_L = true;
                current_pos = q_L;
                break;
            end
        end

        if is_q_L
            break;
        end

        current_edge_start_idx = current_edge_end_idx;
        current_edge_end_idx = mod(current_edge_start_idx, num_vertex) + 1;

        current_pos = obs_vertex(current_edge_start_idx, :);
    end

    % Add path_to_q_L to boundary path
    boundary_path = [boundary_path; path_to_q_L];
end

