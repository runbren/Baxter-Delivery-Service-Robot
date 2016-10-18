
odomSub = rossubscriber('/odometry/filtered');
pause(2)
while(1)
    clc
    disp(odomSub.LatestMessage.Pose.Pose.Position)
    pause(0.2)
end