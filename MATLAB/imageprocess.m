function output = imageprocess(Imsg)
    
    I = readImage(Imsg);
    I = rgb2gray(I);
    I = I(150:260,:);
    
    % Threshold image - manual threshold
    BW = I >25;
    % Invert mask
    BW = imcomplement(BW);
    % Clear borders
    BW = imclearborder(BW);
    % Fill holes
    BW = imfill(BW, 'holes');
    % Erode mask with disk
    radius = 4;
    decomposition = 0;
    se = strel('disk', radius, decomposition);
    BW = imerode(BW, se);
    
    imshow(BW)

    coord = regionprops(BW,'Centroid');
    try
        output = coord.Centroid;
    catch
        disp('No Coordinates')
        output = Inf;
    end

end