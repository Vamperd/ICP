clear;clc;close all;
% read original map
base_ply = pcread('data&demo\0.ply');

% init rotation matrix
tform_init = eye(4,4);

% set Tolerance and MaxIterations
Tolerance = 0.00001;
MaxIterations = 1000;

% point to point icp
% read
str = ['data&demo\1.ply'];  
curr_ply = pcread(str);     % currernt point cloud
    
% icp
% base_ply: base point cloud
    [tform_init, curr_ply] = myicp(curr_ply, base_ply, ...
    tform_init, MaxIterations, Tolerance,'kd');
% [tform_init, curr_ply] = myplicp(curr_ply, base_ply, ...
%      tform_init, MaxIterations, Tolerance);
    
robot_tf = tform_init;

% disp tform
disp(tform_init);

% draw
% point cloud
figure("Name","point cloud");
%base_ply;
pcshowpair(base_ply, curr_ply,'MarkerSize',50);