figure(1)
I = imread('20160912_114922.jpg');
I = imrotate(I,-90);
imshow(I)

IBW = rgb2gray(I);

figure(2)
[sobelBW, sobelthreshold] = edge(IBW,'sobel');
imshow(sobelBW)

segI = segmentImage(IBW);
segIX = createMask(I);
filI = filterRegions(segI);

figure(3)
subplot(1,2,1)
imshow(segI)
subplot(1,2,2)
imshow(filI)

figure(4)
imshow(segIX)
% figure(3)
% [cannyBW, cannythreshold] = edge(IBW,'canny');
% imshow(cannyBW)
% 
% figure(4)
% [logBW, logthreshold] = edge(IBW,'log');
% imshow(logBW)
% 
% figure(5)
% [prewittBW, cannythreshold] = edge(IBW,'canny');
% imshow(cannyBW)