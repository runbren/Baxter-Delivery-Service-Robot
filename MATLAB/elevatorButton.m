function elevatorButton()

    limbPub = rospublisher('/robot/limb/right/joint_command');
    cameraSub=rossubscriber('/cameras/right_hand_camera/image');
    IRSub = rossubscriber('/robot/analog_io/right_hand_range/value_uint32');
    IRData = receive(IRSub);

    imgmsg = receive(cameraSub);
    
    %Move arm into viewing positiong
    [jointCmd, IsValid] = IKService(0.8, -0.2);
    controlRate = robotics.Rate(10);
    if IsValid
        for i=0:50
            send(limbPub,jointCmd);
            waitfor(controlRate);
        end
    else
        disp('Not a valid coordinate')
    end
    

    
    coordinates = imageprocess(imgmsg);
    clc
    disp(coordinates)
    
    deltaY = (coordinates(1)-300)/1500;
    
    %Move arm into pre-pressing positiong
    [jointCmd, IsValid] = IKService(0.8 + IRData.Data/1000-20, -0.2 - deltaY);
    controlRate = robotics.Rate(10);
    if IsValid
        for i=0:50
            send(limbPub,jointCmd);
            waitfor(controlRate);
        end
    else
        disp('Not a valid coordinate')
    end
    
    %Move arm into pressing positiong
    [jointCmd, IsValid] = IKService(0.8 + IRData.Data/1000, -0.2 - deltaY);
    controlRate = robotics.Rate(10);
    if IsValid
        for i=0:50
            send(limbPub,jointCmd);
            waitfor(controlRate);
        end
    else
        disp('Not a valid coordinate')
    end
    
    %Move arm into viewing positiong
    [jointCmd, IsValid] = IKService(0.8, -0.2);
    controlRate = robotics.Rate(10);
    if IsValid
        for i=0:50
            send(limbPub,jointCmd);
            waitfor(controlRate);
        end
    else
        disp('Not a valid coordinate')
    end
    
end
