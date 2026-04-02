clear;clc;close all;
% read original map
base_ply = pcread('data&demo\0.ply');

% init rotation matrix
tform_init = eye(4,4);
robot_tf = cell(10,1);
robot_tf{1} = tform_init;   %robot_tf{1} = eye(4,4);

% set Tolerance and MaxIterations
Tolerance = 0.00001;
MaxIterations = 10000;

tic;
% point to point icp
for i = 1:1:9
    disp(i);
    % read
    str = ['data&demo\', num2str(i) , '.ply'];  
    curr_ply = pcread(str);     % currernt point cloud
    
    % icp
    % base_ply: base point cloud
    [tform_init, curr_ply] = myicp(curr_ply, base_ply, ...
    tform_init, MaxIterations, Tolerance,'kd');
%     [tform_init, curr_ply] = myplicp(curr_ply, base_ply, ...
%      tform_init, MaxIterations, Tolerance);
    
    robot_tf{i+1} = tform_init;

    % merge
    base_ply = pcmerge(base_ply, curr_ply, 0.01);

    % disp tform
    disp(tform_init);
end

% get moving trajectory
x=zeros(10,1);
y=zeros(10,1);
z=zeros(10,1);
for i = 1:1:10
    x(i,:) = robot_tf{i}(1,4);
    y(i,:) = robot_tf{i}(2,4);
    z(i,:) = robot_tf{i}(3,4);
end
t=toc;

% draw
% point cloud
figure("Name","point cloud");
%base_ply;
pcshow(base_ply, 'MarkerSize', 10);

% trajectory
figure("Name","trajectory");
pcshow(base_ply, 'MarkerSize', 10);
hold on;
plot3(x,y,z,'LineWidth',3,'Color','white');
hold off;

disp(t)