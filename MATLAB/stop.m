function stop(velpub, velData)
    velData.Linear.X = 0;
    velData.Linear.Y = 0;
    velData.Linear.Z = 0;
    
    velData.Angular.X = 0;
    velData.Angular.Y = 0;
    velData.Angular.Z = 0;
    
    send(velpub,velData)
    
end