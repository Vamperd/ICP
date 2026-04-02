ply_0 = pcread('ICP_data/0.ply');
ply_1 = pcread('ICP_data/1.ply');

figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);

[tform, ply_1] = pcregistericp(ply_1, ply_0);

figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);