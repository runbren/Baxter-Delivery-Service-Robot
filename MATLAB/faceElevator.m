function faceElevator(odomSub, estimatedPose, velpub, velData)
    
    initOdom = receive(odomSub);
    initodomPose = initOdom.Pose.Pose;
    initOdomZ = angle(initodomPose.Orientation);
    
    Z = estimatedPose(3);
    
    controlRate = robotics.Rate(20);
    
    velData.Linear.X = 0;
    velData.Angular.Z = 0.3;
    
    while Z<1.57        
        send(velpub, velData)
        
        odomData = odomSub.LatestMessage;
        odomOrientation = odomData.Pose.Pose.Orientation;
        odomZ = angle(odomOrientation);
        
        Z = estimatedPose(3) + odomZ-initOdomZ;
        disp(Z)
        
        waitfor(controlRate)
        
    end
    
    stop(velpub, velData)
    
end

function output = angle(orientation)
    quat = [orientation.X orientation.Y orientation.Z orientation.W];
    eul = quat2eul(quat);
    
    output = eul(3);
end