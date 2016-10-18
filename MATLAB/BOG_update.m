map = robotics.BinaryOccupancyGrid(45,20,40);
map.GridLocationInWorld = [-2,-7];

%show(map)

laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odometry/filtered');
disp('pausing')
pause(4)
% Loop over all the messages
while(1)
    % Define the Quaternion in a vector form
    odomOrientation = odomSub.LatestMessage.Pose.Pose.Orientation;
    q = [odomOrientation.W odomOrientation.X odomOrientation.Y odomOrientation.Z];

    % Convert the Quaternion into Euler angles
    orientation = quat2eul(q, 'ZYX');
    
    numAngles = length(laserSub.LatestMessage.Ranges);
    angleMax =  laserSub.LatestMessage.AngleMax;
    angleMin =  laserSub.LatestMessage.AngleMin;
    increment = (abs(angleMax)+abs(angleMin))/numAngles;
    angles = transpose(angleMin:increment:angleMax-increment);
    ranges = laserSub.LatestMessage.Ranges;
    laserRanges = double([ranges angles]);
    % The z-direction rotation component is the heading of the
    % robot. Use the heading value along with the z-rotation axis to define
    % the rotation matrix
    rotMatrixAlongZ = axang2rotm([0 0 1 orientation(1)]);

    % Convert sensor values (originally in the robot's frame) into the
    % world coordinate frame. This is a 2D coordinate transformation, so
    % only the first two rows and columns are used from the rotation matrix.
    rangesInWorldFrame = rotMatrixAlongZ(1:2,1:2) * laserRanges';
    rangesInworldFame
    % Update map based on laser scan data in the world coordinate frame.
    % Populate the map, using the setOccupancy function, by setting the
    % obstacle location on the map as occupied for every sensor reading that
    % detects obstacles
    setOccupancy(map, rangesInWorldFrame', 1);
end