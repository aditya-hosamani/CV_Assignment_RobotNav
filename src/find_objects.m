% clear all; close all; clc;
% 
% img = imread("../test_images/images/img1.png");
% x = find_objects(img, "red");

function coords = find_objects(img, cube_color)
% FIND_OBJ Find the location of the cubes and targets,
% and the pose of the robot using color tresholding
%
%    Inputs:
%        img: an image containing the current arrangement of robot and blocks.
%        calib: calibration results from function calibrate.
%
%    Output:
%        coords: matrix containing the coordinates of cube and target
%        centroids and the centroids of the cyan and magenta dots on
%        top of the robot

    [cyan, magenta] = locate_robot(img);
    coords = [];

    if (cube_color == "red")
        [rcube, rtarget] = locate_red(img);
        coords = [transpose(cyan) transpose(magenta) transpose(rcube) transpose(rtarget)];
        %coords(3) = transpose(rcube);
        %coords(4) = transpose(rtarget);
    elseif (cube_color == "green")
        [gcube, gtarget] = locate_green(img);
        coords = [transpose(cyan) transpose(magenta) transpose(gcube) transpose(gtarget)];
        %coords(3) = transpose(gcube);
        %coords(4) = transpose(gtarget);
    elseif (cube_color == "blue")
        [bcube, btarget] = locate_blue(img);
        coords = [transpose(cyan) transpose(magenta) transpose(bcube) transpose(btarget)];
        %coords(3) = transpose(bcube);
        %coords(4) = transpose(btarget);
    end

%     disp("Cyan");disp(cyan);
%     disp("magenta");disp(magenta);
%     disp("rcube");disp(rcube);
%     disp("rtarget");disp(rtarget);

    %coords = [transpose(cyan) transpose(magenta) transpose(rcube) ...
    %    transpose(gcube) transpose(bcube) transpose(rtarget) ...
    %    transpose(gtarget) transpose(btarget)];
end

function [rcube, rtarget] = locate_red(img)
    % Locate red in RGB color space
    % Specify minimum and maximum values for color channels
    rmin = 50;  
    rmax = 255;
    gmin = 0;
    gmax = 50;
    bmin = 0;
    bmax = 55;

    [rcube, rtarget] = locate_cube_and_target(img, rmin, rmax, gmin, gmax, bmin, bmax);
end

function [gcube, gtarget] = locate_green(img)
    % Locate green in RGB color space
    rmin = 0;  
    rmax = 50;
    gmin = 57;
    gmax = 255;
    bmin = 0;
    bmax = 40;

    [gcube, gtarget] = locate_cube_and_target(img, rmin, rmax, gmin, gmax, bmin, bmax);
end

function [bcube, btarget] = locate_blue(img)
    % Locate blue in RGB color space
    rmin = 0;  
    rmax = 70;
    gmin = 0;
    gmax = 50;
    bmin = 35;
    bmax = 255;

    [bcube, btarget] = locate_cube_and_target(img, rmin, rmax, gmin, gmax, bmin, bmax);
end

function [cube_centroid, target_centroid] =locate_cube_and_target(img, rmin, rmax, gmin, gmax, bmin, bmax)

    filter = (img(:, :, 1) >= rmin) & (img(:, :, 1) <= rmax) & ...
      (img(:, :, 2) >= gmin) & (img(:, :, 2) <= gmax) & ...
      (img(:, :, 3) >= bmin) & (img(:, :, 3) <= bmax);

    %colored_area = filter      % This line is for testing

    % Remove small areas from the binary image and fill holes in areas
    colored_area = bwareaopen(filter, 300);
    colored_area = imfill(colored_area, "holes");

    props = regionprops('table', colored_area, 'Centroid', 'Circularity', ...
        'MajorAxisLength','MinorAxisLength', 'Area', 'Eccentricity');

    max_val = max(props.Circularity);
    target_idx = find(props.Circularity == max_val);
    target_centroid = props.Centroid(target_idx, :);

    cube_idx = find(min([props.MajorAxisLength] / [props.MinorAxisLength]));
    cube_centroid = props.Centroid(cube_idx, :);

    % Plotting for testing, to be deleted later
    %figure;
    %imshow(colored_area);
    %hold on;
    %plot(target_centroid(1,1), target_centroid(1,2), "diamond", 'MarkerSize', 8, 'markerFaceColor', "red");
    %plot(cube_centroid(1,1), cube_centroid(1,2), "o", 'MarkerSize', 8, 'markerFaceColor', "red");
    %hold off;
end

function [cyan_centroid, magenta_centroid] = locate_robot(img)
    % Locate cyan in RGB color space
    rmin_c = 0;  
    rmax_c = 100;
    gmin_c = 50;
    gmax_c = 150;
    bmin_c = 100;
    bmax_c = 200;

    cyan_centroid = locate_dot(img, rmin_c, rmax_c, gmin_c, gmax_c, bmin_c, bmax_c);

    % Locate magenta
    rmin_m = 100;  
    rmax_m = 255;
    gmin_m = 0;
    gmax_m = 70;
    bmin_m = 100;
    bmax_m = 255;

    %magenta_centroid = locate_dot(img, rmin_m, rmax_m, gmin_m, gmax_m, bmin_m, bmax_m);
    magenta_centroid = cyan_centroid;
end

function dot_centroid = locate_dot(img, rmin, rmax, gmin, gmax, bmin, bmax)
    filter = (img(:, :, 1) >= rmin) & (img(:, :, 1) <= rmax) & ...
      (img(:, :, 2) >= gmin) & (img(:, :, 2) <= gmax) & ...
      (img(:, :, 3) >= bmin) & (img(:, :, 3) <= bmax);

    colored_area = bwareaopen(filter, 50);
    colored_area = imfill(colored_area, "holes");

    props = regionprops('table', colored_area, 'Centroid', 'Circularity', ...
        'MajorAxisLength','MinorAxisLength', 'Area', 'Eccentricity');

    max_val = max(props.Circularity);
    dot_idx = find(props.Circularity == max_val);
    dot_centroid = props.Centroid(dot_idx, :);

    % For the arrow on top of the robot, delete if we will not use the arrow
    %display(size(props))
    %arrow = [];
    %for i = 1:size(props)
    %    if (props.Circularity(i, :) > 0.85)
    %        arrow = [arrow, props.Centroid(i, :)];
    %    end
    %end

    %figure;
    %imshow(colored_area);
    %hold on;
    %plot(dot_centroid(1,1), dot_centroid(1,2), "diamond", 'MarkerSize', 3, 'markerFaceColor', "red");
    %hold off;
end
