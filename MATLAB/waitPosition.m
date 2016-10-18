function waitPosition(velpub, velData, odomSub)

    odomData = receive(odomSub);
    
    initPose = odomData.Pose.Pose.Position;
    
    X = -1.5;
    Y = -0.8;
    poseX = 0;
    poseY = 0;
        
    while (poseX>X && poseY>Y)
        poseX = odomSub.LatestMessage.Pose.Pose.Position.X - initPose.X;
        poseY = odomSub.LatestMessage.Pose.Pose.Position.Y - initPose.Y;
        
        if poseX>X
            velData.Linear.X = -0.3;
        else
            velData.Linear.X = 0;
        end
        
        if poseY>Y
            velData.Linear.Y = 0.3;
        else
            velData.Linear.Y = 0;
        end
        send(velpub, velData)
        
    end
end