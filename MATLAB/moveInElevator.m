function moveInElevator(laserSub, velpub, velData, VFH)

    lenranges = length(laserSub.LatestMessage.Ranges);
    velData.Linear.X = 0.4;
    velData.Angular.Z = 0;
    targetDir = 0;
    
    controlRate = robotics.Rate(10);
    while laserSub.LatestMessage.Ranges(round(lenranges/2))>0.1

        ranges = double(laserSub.LatestMessage.Ranges);
        angles = double(laserSub.LatestMessage.readScanAngles);


        steeringDir = step(VFH, ranges, angles, targetDir);
        velData.Linear.Y = steeringDir;
        clc
        disp('steeringDir & Y velocity')
        disp(steeringDir)
        send(velpub,velData)
        figure(1)
        show(VFH)
        
        waitfor(controlRate)
    end

end