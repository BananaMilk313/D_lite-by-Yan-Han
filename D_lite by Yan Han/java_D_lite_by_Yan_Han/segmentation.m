function [robot_xy, target_xy, map] = segmentation(path, thr1, thr2, thr3)
% function [robot_xy, target_xy, map] = segmentation(path, thr1, thr2, thr3)
%
% Function that loads an image from the specified path and performs operations
% on the image to obtain the coordinates of the robot, the target coordinates,
% and a matrix containing all obstacles.

% Read the image and convert it to grayscale
img = rgb2gray(imread(path));

% Create a structuring element
se = strel('disk', 4);

% Convert the image to binary based on threshold values
im_robot = im2bw(img, thr2);
im_target = im2bw(img, thr1);
im_target = imerode(im_target, se); % Erode the target image
im_obst = im2bw(img, thr3);

% Close the obstacles image
im_obst = imclose(im_obst, se);

% Complement the obstacles image and subtract the target image
im_obst = imcomplement(imcomplement(im_obst) - imcomplement(im_target));
im_target = imcomplement(im_target);

% Label connected components in the robot image
[label, num] = bwlabel(im_robot, 4);

% Check if any objects were detected
if num
    % Get the properties of the detected objects
    object_properties = regionprops(label, 'centroid', 'Orientation');
    robot_xy = cat(1, object_properties.Centroid); % Get the robot coordinates
else
    fprintf('No robot detected in the image\n')
    return;
end

% Label connected components in the target image
[label, num] = bwlabel(im_target, 4);

% Check if any objects were detected
if num
    % Get the properties of the detected objects
    object_properties = regionprops(label, 'centroid', 'Orientation');
    target_xy = cat(1, object_properties.Centroid); % Get the target coordinates
else
    fprintf('No target detected in the image\n')
    return;
end

% Set the map to the obstacles image
map = im_obst;
end