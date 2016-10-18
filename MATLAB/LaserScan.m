
laser = rossubscriber('/scan');
y1=[-0.6:0.01:0.6];
x1=[zeros(1,121)+0.5];
y2=[zeros(1,101)+0.6];
x2=[-0.5:0.01:0.5];
y3=[zeros(1,101)-0.6];
x3=[-0.5:0.01:0.5];
%camera = rossubscriber('/cameras/head_camera/image');
figure
while(1);
    scandata = receive(laser,10);
    %image = receive(camera,10);
    
    plot(scandata,'MaximumRange',6);
    %show(image)
    hold on
    plot(x1,y1,x2,y2,x3,y3)
    hold off
end