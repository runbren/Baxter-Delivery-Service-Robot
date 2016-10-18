function output = BOG(x,y,res)

output = robotics.BinaryOccupancyGrid(x,y,res);
%x = [0.2;0.3;0.3; 0.4; 0.5; 0.6];
%y = [0.5;0.5;0.5;0.5;0.5;0.5;];

%setOccupancy(map, [x y], ones(6,1))
%figure
%show(map)