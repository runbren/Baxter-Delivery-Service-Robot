function waitDoor(laserSub)

    ranges = laserSub.LatestMessage.Ranges;
    rangePosition = round(length(ranges)/2);
    initrange = ranges(rangePosition);
    range = initrange;
    
    controlRate = robotics.Rate(10);
    while (range < (initrange+0.5))
        
        ranges = laserSub.LatestMessage.Ranges;
        range = ranges(rangePosition);
        
        waitfor(controlRate);
    end
end