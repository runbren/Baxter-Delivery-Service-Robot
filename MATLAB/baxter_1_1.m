my_laptop_IP = '192.168.';                             %IP address of the computer running MATLAB
global_MATLAB_node_name = '/matlab_global_node';            %Unique name for the MATLAB node (optional)
gazeboVM_IP = '192.168.102.128';                            %IP address of the virtual machine

rosshutdown;                                                %Shutdown any potentially running ROS

rosinit(gazeboVM_IP, 'NodeHost', my_laptop_IP, ...
                     'NodeName', global_MATLAB_node_name);  %Initiate ROS node for MATLAB

velPub = rospublisher('/mobile_base/commands/velocity'); 
velData = rosmessage(velPub);

odom = rossubscriber('/odom');
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;


map = BOG(1,1,10);
prm = robotics.PRM;
prm.Map = map;
prm.NumNodes = 20;
prm.ConnectionDistance = 0.5;

startLocation = [0.1 0.1];
endLocation = [0.9 0.9];

path = findpath(prm,startLocation,endLocation);

controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity = 0.3;
controller.LookaheadDistance = 0.2;

goalRadius = 0.1;
robotCurrentLocation = path(1,:);
distanceToGoal = norm(robotCurrentLocation - endLocation);

controlRate = robotics.Rate(10);
show(prm)

x = 0;
y = 0;
while( distanceToGoal > goalRadius )
        
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = rad2deg(angles(1));
    
    currentPose = [x , y, theta];
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = step(controller, currentPose);
    
    velData.Linear.X = v;
    velData.Angular.Z = omega;
    
    % Simulate the robot using the controller outputs.
    %drive(robot, v, omega);
    

    pose = odomdata.Pose.Pose;
    pose.Position.X = x;
    pose.Position.Y = y;
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getRobotPose();

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);

end

show(prm)