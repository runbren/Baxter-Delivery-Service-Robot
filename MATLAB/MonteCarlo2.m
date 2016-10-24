BaxterROS_Connection
XB_2_BOG
PRM_mb

disp('odom and rangefinder')
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.5 0.5 0.5 0.5]; %[Rot error due to Rot motion, Rot error due to trans motion, Trans error due to trans motion, Trans error due to rot motion]
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.2 6];
rangeFinderModel.Map = map;

%%
% disp('tftree')
% % Query the Transformation Tree (tf tree) in ROS.
% tftree = rostf;
% waitForTransform(tftree,'/base_footprint','/base');
% sensorTransform = getTransform(tftree,'/base_footprint', '/base');
% %sTTransform = sensorTransform.Transform;
% % Get the euler rotation angles.
% baseQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
%     sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
% baseRotation = quat2eul(baseQuat, 'ZYX');
% 
% laserX = sensorTransform.Transform.Translation.X;
% laserY = sensorTransform.Transform.Translation.Y + 0.3; % Sensor is 0.3m in front of /base location
% laserTheta = baseRotation;

% Setup the |SensorPose|, which includes the transla+tion along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = [0.3 0 0];

%%
disp('subscribing')
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odometry/filtered');

disp('publishing')
velData = rosmessage('geometry_msgs/Twist');
velpub = rospublisher('/mobility_base/cmd_vel');

%%
disp('creating VFH')

VFH = robotics.VectorFieldHistogram;
VFH.NumAngularSectors = length(laserSub.LatestMessage.Ranges);
VFH.DistanceLimits = [0.06 1.2];
VFH.RobotRadius = 0.3;
VFH.MinTurningRadius = 0.1;


%%
disp('creating amcl')
amcl = robotics.MonteCarloLocalization;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.1,0.1,0.01];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 4000];
amcl.GlobalLocalization = false;
amcl.InitialPose = [0 0 0]; %[x y theta (radians)]
amcl.InitialCovariance = eye(3)*1;

visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Controller

    controller = robotics.PurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 0.3;
    controller.MaxAngularVelocity = 0.3;
    controller.LookaheadDistance = 1;

goalRadius = 0.1;
robotStartLocation = amcl.InitialPose(1:2);
endLocation = [26.5 4.2];
distanceToGoal = norm(robotStartLocation - endLocation);

controlRate = robotics.Rate(6);

%%

while ( distanceToGoal > goalRadius )

    
    %Record the old pose if it exists
    try
        posePast = pose;
    catch
        
    end
    
    % Receive laser scan and odometry message.
    scan = receive(laserSub);
    odompose = odomSub.LatestMessage;

    % Convert range and angle readings to double type for using with
    % AMCL step function.
    ranges = double(scan.Ranges);
    angles = double(scan.readScanAngles);

    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    
    
        
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = step(amcl, pose, ranges, angles);
    
    
    currentPose = estimatedPose;
    
%     if isUpdated
%         disp('AMCL control')
%         currentPose = estimatedPose;
% 
%     else
%         if posePast
%             disp('odom control')
%             poseDiff = pose - posePast;
%             currentPose = estimatedPose + poseDiff;
% 
%             pose = posePast;
%         else 
%             currentPose = pose;
%         end
%     end


    if isUpdated
        plotStep(visualizationHelper, amcl, estimatedPose, ranges, angles,1)
    end
    
    %update distance to the goal
    distanceToGoal = norm(estimatedPose(1:2) - endLocation);
    clc
    disp('Distance to Goal')
    disp(distanceToGoal)

    %set control velocities
    [v, theta] = step(controller, estimatedPose);
    disp('theta')
    disp(theta)
    disp('estimated pose')
    disp(estimatedPose)
    theta = 0;
    steeringDir = step(VFH, ranges, angles, theta);
    
    figure(4)
    show(VFH)
    if isnan(steeringDir)
        velData.Linear.X = 0;
    else
        velData.Linear.X = v;
    end
    velData.Angular.Z = steeringDir;
        
    
    if distanceToGoal<2
        controller.DesiredLinearVelocity = 0.1;
    end
    send(velpub,velData)
    
    waitfor(controlRate);
end

stop(velpub, velData)

faceElevator(odomSub, estimatedPose, velpub, velData)

elevatorButton()

waitPosition(velpub, velData, odomSub)

waitDoor(laserSub)

moveInElevator(laserSub, velpub, velData, VFH)

