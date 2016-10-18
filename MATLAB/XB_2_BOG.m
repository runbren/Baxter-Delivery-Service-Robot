image =imread('/FP-KW-XB-G-4 ver 2.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 100;
map = robotics.BinaryOccupancyGrid(bwimage,43.12);
map.GridLocationInWorld = [-3 -2];
% show(map)

%%
% image =imread('FP-KW-XB-G-4 ver 2 PRM.png');
% grayimage = rgb2gray(image);
% bwimage = grayimage < 100;
% map = robotics.BinaryOccupancyGrid(bwimage,43.12);
% map.GridLocationInWorld = [-3 -2];
% show(map)