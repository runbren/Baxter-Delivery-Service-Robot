disp('creating PRM')
prm = robotics.PRM;

%This is commented out for saving time when initialising. Inflating takes a
%long time.
% robotRadius = 0.6;
% disp('copying map')
% mapInflated = copy(map);
% disp('inflating map')
% inflate(mapInflated,robotRadius);
load('mapInflated.mat');

disp('setting map')
prm.Map = mapInflated;
disp('setting nodes')
prm.NumNodes = 200;
disp('setting distance')
prm.ConnectionDistance = 6;

startLocation = [0 0];
endLocation = [26.5 4.2];

disp('finding path')
path = findpath(prm,startLocation,endLocation);

disp('showing PRM')

% load('prm.mat');
% load('path.mat');
figure()
show(prm)