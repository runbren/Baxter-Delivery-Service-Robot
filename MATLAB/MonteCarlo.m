
disp('odom and rangefinder')
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.2 6];
rangeFinderModel.Map = map;

%%
disp('tftree')
% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree,'/base_footprint','/base');
sensorTransform = getTransform(tftree,'/base_footprint', '/base');
%sTTransform = sensorTransform.Transform;
% Get the euler rotation angles.
%laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
%    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
%laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y 0];

%%
disp('subscribing')
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odometry/filtered');

disp('publishing')
velData = rosmessage('geometry_msgs/Twist');
velpub = rospublisher('/mobility_base/cmd_vel');

%%

disp('creating amcl')
amcl = robotics.MonteCarloLocalization;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.InitialPose = [3 2 0]; %[x y theta (radians)]
amcl.InitialCovariance = eye(3)*1;

visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Controller

controller = createController();
% controller = robotics.PurePursuit;
% controller.Waypoints = path;
% controller.DesiredLinearVelocity = 0.1;
% controller.MaxAngularVelocity = 0.3;
% controller.LookaheadDistance = 0.2;

goalRadius = 0.1;
robotCurrentLocation = path(1,:);
endLocation = [29 6.5];
distanceToGoal = norm(robotCurrentLocation - endLocation);

controlRate = robotics.Rate(10);

%%

while ( distanceToGoal > goalRadius )

    % Receive laser scan and odometry message.
    scan = receive(laserSub);
    odompose = odomSub.LatestMessage;

    % Convert range and angle readings to double type for using with
    % AMCL step function.
    ranges = double(scan.Ranges);
    angles = double(scan.readScanAngles);
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.

    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    
    %Record the old pose if it exists
    try
        posePast = pose;
    catch
        
    end
        
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = step(amcl, pose, ranges, angles);

    
    if isUpdated
        currentPose = estimatedPose;
    else
        try 
            currentPose = odomAddition(estimatedPose, posePast);
            pose = posePast;
        catch
            
        end
            
    end
    
    %update distance to the goal
    distanceToGoal = norm(currentPose(1:2) - endLocation);
    
    %set control velocities
    [v, omega] = step(controller, currentPose);
    
    velData.Linear.X = v;
    velData.Angular.Z = omega;
    %disp(velData)
    
    %send(velpub,velData) Commented out for remote control

    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        plotStep(visualizationHelper, amcl, estimatedPose, ranges, angles, i)
    end
    
    waitfor(controlRate);
end

function output = odomAddition(estimatedPose, posePast)
    

    output = 2;
end

function output = createController()
    controller = robotics.PurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 0.1;
    controller.MaxAngularVelocity = 0.3;
    controller.LookaheadDistance = 0.2;

    output = controller;
end