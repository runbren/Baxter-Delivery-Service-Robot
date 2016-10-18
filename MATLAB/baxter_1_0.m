map = BOG(1,1,10);
prm = robotics.PRM;
prm.Map = map;

startLocation = [0.1 0.1];
endLocation = [0.9 0.9];

path = findpath(prm,startLocation,endLocation)
show(map)