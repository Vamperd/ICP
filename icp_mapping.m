clear;clc;

laser_map = pcread('ICP_data/0.ply');

tform_init = affine3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);

robot_tf{1} = tform_init;

for i = 1:1:9
    
    disp(i);
    
    % read
    str = ['ICP_data/' ,num2str(i) , '.ply'];  
    curr_ply = pcread(str);
    
    % icp

    [tform_init, curr_ply] = pcregistericp(curr_ply, laser_map, 'Metric','pointToPoint', 'InitialTransform', tform_init, 'MaxIterations', 100, 'Tolerance', [0.01, 0.001]);
    robot_tf{i+1} = tform_init;
    
    % merge
    laser_map = pcmerge(laser_map, curr_ply, 0.01);
   
end

figure;
pcshow(laser_map, 'MarkerSize', 20);
title('ICP Baseline Map');

% 将结果图保存至 results_matlab 文件夹
saveas(gcf, 'results_matlab/icp_baseline_map.png');

% 暂停脚本运行，直到用户按下任意键（或者使用 pause(5) 停留5秒）
pause(5);