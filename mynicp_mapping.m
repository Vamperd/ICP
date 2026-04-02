clear;
clc;
close all;

% 简洁版 N-ICP 主程序，沿用和 syk 一样的“累计地图 + 上一时刻位姿初值”流程
data_dir = fullfile(pwd, 'ICP_data');
output_dir = fullfile(pwd, 'results_matlab', 'nicp');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

laser_map = pcread(fullfile(data_dir, '0.ply'));
tform_init = eye(4);
robot_tf = cell(10, 1);
robot_tf{1} = tform_init;
MSE = cell(9, 1);
metrics = zeros(9, 3);

for i = 1:9
    fprintf('mynicp mapping: pair %d -> %d\n', i - 1, i);
    curr_ply = pcread(fullfile(data_dir, sprintf('%d.ply', i)));

    [tform_init, curr_ply, debug_info] = mynicp(curr_ply, laser_map, tform_init, 50, 0.005, 0.5, 0.5);
    robot_tf{i + 1} = tform_init;
    MSE{i} = debug_info.mse_history;
    metrics(i, :) = [debug_info.iteration_count, debug_info.valid_correspondence_count, debug_info.mean_residual];

    laser_map = pcmerge(laser_map, curr_ply, 0.01);
end

[global_poses, metrics_table] = build_tables(robot_tf, metrics);
disp('================ My N-ICP Metrics ================');
disp(metrics_table);
disp('================ My N-ICP Global Poses ================');
disp(global_poses);

save(fullfile(output_dir, 'result_summary.mat'), 'robot_tf', 'global_poses', 'metrics_table', 'MSE');

plot_map_and_trajectory(laser_map, robot_tf, fullfile(output_dir, 'local_map.png'), 'My N-ICP 地图与轨迹');
plot_topdown_trajectory(robot_tf, fullfile(output_dir, 'trajectory_topdown.png'), 'My N-ICP 俯视轨迹图');
plot_mse_overview(MSE, metrics_table, fullfile(output_dir, 'mse_overview.png'), 'My N-ICP 9 次配准 MSE');

function [global_poses, metrics_table] = build_tables(robot_tf, metrics)
    global_x = zeros(10, 1);
    global_y = zeros(10, 1);
    global_theta = zeros(10, 1);
    frame_names = cell(10, 1);

    for i = 1:10
        global_x(i) = robot_tf{i}(1, 4);
        global_y(i) = robot_tf{i}(2, 4);
        global_theta(i) = atan2(robot_tf{i}(2, 1), robot_tf{i}(1, 1));
        frame_names{i} = sprintf('frame_%d', i - 1);
    end

    pair_names = cell(9, 1);
    for i = 1:9
        pair_names{i} = sprintf('pair_%d_to_%d', i - 1, i);
    end

    global_poses = table(global_x, global_y, global_theta, 'RowNames', frame_names);
    metrics_table = table(metrics(:, 1), metrics(:, 2), metrics(:, 3), ...
        'VariableNames', {'iteration_count', 'valid_correspondence_count', 'mean_residual'}, ...
        'RowNames', pair_names);
end

function plot_map_and_trajectory(laser_map, robot_tf, output_file, fig_title)
    x = zeros(10, 1);
    y = zeros(10, 1);
    z = zeros(10, 1);
    for i = 1:10
        x(i) = robot_tf{i}(1, 4);
        y(i) = robot_tf{i}(2, 4);
        z(i) = robot_tf{i}(3, 4);
    end

    fig = figure('Visible', 'off', 'Color', 'w');
    pcshow(laser_map, 'MarkerSize', 20);
    hold on;
    plot3(x, y, z, 'r*-', 'LineWidth', 1.5);
    title(fig_title);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    exportgraphics(fig, output_file, 'Resolution', 220);
    close(fig);
end

function plot_topdown_trajectory(robot_tf, output_file, fig_title)
    x = zeros(10, 1);
    y = zeros(10, 1);
    for i = 1:10
        x(i) = robot_tf{i}(1, 4);
        y(i) = robot_tf{i}(2, 4);
    end

    fig = figure('Visible', 'off', 'Color', 'w');
    plot(x, y, 'm*-', 'LineWidth', 1.2);
    hold on;
    for i = 1:10
        text(x(i), y(i), sprintf(' %d', i - 1), 'FontSize', 9);
    end
    grid on;
    xlim([0, 0.5]);
    xlabel('x');
    ylabel('y');
    title(fig_title);
    exportgraphics(fig, output_file, 'Resolution', 220);
    close(fig);
end

function plot_mse_overview(MSE, metrics_table, output_file, fig_title)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1200, 900]);
    for i = 1:9
        subplot(3, 3, i);
        plot(MSE{i}, 'LineWidth', 1.2);
        grid on;
        title(sprintf('pair %d->%d | iter=%d | match=%d | res=%.4f', ...
            i - 1, i, metrics_table.iteration_count(i), ...
            metrics_table.valid_correspondence_count(i), metrics_table.mean_residual(i)));
        xlabel('迭代次数');
        ylabel('MSE');
    end
    sgtitle(fig_title);
    exportgraphics(fig, output_file, 'Resolution', 220);
    close(fig);
end
