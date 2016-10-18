

camera = rossubscriber('/cameras/head_camera/image');
figure
pause(1)
while(1);
    %cameramsg = receive(camera,10);
    cameramsg = camera.LatestMessage;
    img = readImage(cameramsg);
    
    imshow(img);
end