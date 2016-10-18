

camera = rossubscriber('/cameras/head_camera/image');
figure
while(1);
    cameramsg = receive(camera,10);
    img = readImage(cameramsg);
    
    imshow(img);
end