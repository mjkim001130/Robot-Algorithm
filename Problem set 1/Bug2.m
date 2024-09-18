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
title('Bug 2');

plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start point
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);   % Goal point
plot([start(1), goal(1)], [start(2), goal(2)], 'k--', 'LineWidth', 2);

% Obstacles
obstacle1 = [3, 8; 5, 8; 8, 5; 5, 2; 3, 2; 5,5]; 
obstacle2 = [10, 3; 14, 3; 14, 7; 10, 7];


obstacles = {obstacle1, obstacle2};

for i = 1:length(obstacles)
    obs = obstacles{i};
    fill(obs(:,1), obs(:,2), [0.5 0.5 0.5]); 
end

xlim([-1, 20]);
ylim([-1, 12]);

%% Initialize the bug 

current_pos = start;
path = current_pos; 

bug_plot = plot(current_pos(1), current_pos(2), 'bo', 'MarkerSize', 5, 'LineWidth', 2);

max_step = 100000;
step_size = 0.05;

%% Main

m_line_direction = (goal - start) / norm(goal - start);

for step = 1 : max_step
    % Find the goal
    if norm(goal - current_pos) < step_size * 2
        disp('Find the Goal!!!!!!!!!!!!!!!!!');
        break;
    end

    % Motion-to-goal 
    direction = m_line_direction;
    next_pos = current_pos + direction * step_size;

    collision = false; % Trigger
    for i = 1:length(obstacles)
        obs = obstacles{i};
        num_edge = size(obs, 1);
        for j = 1:num_edge
            x_edge = [obs(j, 1), obs(mod(j, num_edge)+1, 1)]; 
            y_edge = [obs(j, 2), obs(mod(j, num_edge)+1, 2)]; 

            x_line = [current_pos(1), next_pos(1)];
            y_line = [current_pos(2), next_pos(2)];

            % Check collision
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
        path = [path; current_pos]; % Add current_pos to path

        % Update plot
        set(bug_plot, 'XData', current_pos(1), 'YData', current_pos(2));

        if size(path, 1) > 1
            plot(path(end-1:end,1), path(end-1:end,2), 'b', 'LineWidth',2);
        end
        drawnow;
        pause(0.001);
    else
        %% Boundary following
        disp('Collision! Beginning boundary following');
        % Plot hit point
        plot(q_H(1), q_H(2), 'ko', 'MarkerSize', 8, 'LineWidth',2);

        % Start boundary following
        [current_pos, boundary_path] = boundary_following(q_H, goal, collided_obs, step_size, bug_plot, edge_start_idx, start, current_pos);

        % Add boundary_path to path
        path = [path; boundary_path];
    end
end

%% Final Path Plot

plot(path(:,1), path(:,2), 'r', 'LineWidth',2);

hold off;

%% Function: Boundary Following

function [current_pos, boundary_path, no_path_flag] = boundary_following(current_pos, goal, obstacle, step_size, bug_plot, edge_start_idx, start, hit_point)
    boundary_path = [];
    num_vertex = size(obstacle, 1);

    % M-line 
    m_line_direction = (goal - start) / norm(goal - start);
    
    % Distance from start to hit point 
    hit_point_distance = dot(hit_point - start, m_line_direction);

    finish_loop = false;
    no_path_flag = false;
    current_edge_start_idx = edge_start_idx;
    current_edge_end_idx = mod(current_edge_start_idx, num_vertex) + 1;

    boundary_step_size = step_size / 2;

    while ~finish_loop
        start_vertex = obstacle(current_edge_start_idx, :);
        end_vertex = obstacle(current_edge_end_idx, :);

        edge_vector = end_vertex - start_vertex;
        edge_length = norm(edge_vector);
        edge_direction = edge_vector / edge_length;

        remaining_vector = end_vertex - current_pos;
        remaining_length = norm(remaining_vector);
        remaining_direction = remaining_vector / remaining_length;

        num_steps = ceil(remaining_length / boundary_step_size);

        % Following the edge
        for s = 1:num_steps
            current_pos = current_pos + remaining_direction * boundary_step_size;
            boundary_path = [boundary_path; current_pos];

            % Update bug
            set(bug_plot, 'XData', current_pos(1), 'YData', current_pos(2));
            if size(boundary_path, 1) > 1
                plot(boundary_path(end-1:end,1), boundary_path(end-1:end,2), 'r', 'LineWidth',2);
            end
            drawnow;
            pause(0.001);

            % y축 거리로 m_line과의 거리 계산
            distance_to_mline = abs(current_pos(2) - start(2));

            if distance_to_mline < 0.05
                % Compute distance from start to current position along M-line
                current_distance = dot(current_pos - start, m_line_direction);

                % Check the goal is closer than hit point
                if current_distance > hit_point_distance
                    finish_loop = true;
                    disp('The point is on m-line. Change to motion-to-goal');
                    plot(current_pos(1), current_pos(2), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
                    break;
                end
            end

            % 다시 hit point로 돌아오면
            if norm(current_pos - hit_point) < boundary_step_size && size(boundary_path,1) > num_steps
                disp('No path to goal.');
                finish_loop = true;
                no_path_flag = true;
                break;
            end
        end

        if finish_loop
            break;
        end

        % Move to next edge
        current_edge_start_idx = mod(current_edge_start_idx, num_vertex) + 1;
        current_edge_end_idx = mod(current_edge_start_idx , num_vertex) + 1;

    end
end
