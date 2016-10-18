function elevatorButton(velpub, velData)

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
    
    while 1
        coordinates = imageprocess(imgmsg);
        clc
        disp(coordinates)
        waitfor(controlRate)
    end
    
end
    %Move towards the button
%     while (IRSub.LatestMessage.Data>250 || coordinates(2)>21.7 || coordinates(2)<22.2)
%         coordinates = imageprocess(imgmsg);
%         disp(coordinates)
%         disp(IRSub.LatestMessage.Data)
%         imgmsg = cameraSub.LatestMessage;
%         if length(coordinates)==2
%             if coordinates(2)<21.7
%                 velData.Linear.Y = -0.10;
%             elseif coordinates(2)>22.2
%                 velData.Linear.Y = 0.1;
%             else
%                 velData.Linear.Y = 0;
%             end
%         else
%             velData.Linear.Y = 0;
%             coordinates = [0 22];
%         end
%         if IRSub.LatestMessage.Data>250
%             velData.Linear.X = 0.1;
%         else
%             velData.Linear.X = 0;
%         end
%         velData.Angular.Z = 0;
%         send(velpub, velData)
%         disp(velData.Linear)
%     end
    
        %Move arm into button positiong
%     [jointCmd, IsValid] = IKService(-0.2,1.0);
%     controlRate = robotics.Rate(10);
%     if IsValid
%         for i=0:50
%             send(limbPub,jointCmd);
%             waitfor(controlRate);
%         end
%     else
%         disp('Not a valid coordinate')
%     end