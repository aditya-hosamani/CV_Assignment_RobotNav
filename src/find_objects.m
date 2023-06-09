%clear all; close all; clc;
 
%img = imread("..\test_images\new_robot_cover\img16.png");
%x = find_obj(img, "green");

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
    %robot = locate_robot_arrow(img);

    [rcube, rtarget] = locate_red(img);
    [gcube, gtarget] = locate_green_hsv(img);
    [bcube, btarget] = locate_blue_hsv(img);

    coords = [transpose(cyan) transpose(magenta) ...
        transpose(rcube) transpose(gcube) transpose(bcube) ...
        transpose(rtarget) transpose(gtarget) transpose(btarget)]
    %coords = 0;
end

function [rcube, rtarget] = locate_red(img)
    % Locate red in RGB color space
    % Specify minimum and maximum values for color channels
    rmin = 55;
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
    gmin = 50;
    gmax = 255;
    bmin = 0;
    bmax = 50;

    [gcube, gtarget] = locate_cube_and_target(img, rmin, rmax, gmin, gmax, bmin, bmax);
end

function [bcube, btarget] = locate_blue(img)
    % Locate blue in RGB color space
    rmin = 0;  
    rmax = 60;
    gmin = 0;
    gmax = 60;
    bmin = 30;
    bmax = 255;

    [bcube, btarget] = locate_cube_and_target(img, rmin, rmax, gmin, gmax, bmin, bmax);
end

function [gcube, gtarget] = locate_green_hsv(img)
    % Locate green in HSV color space
    img = rgb2hsv(img);

    hmin = 0.240;  
    hmax = 0.384;
    smin = 0.192;
    smax = 1.000;
    vmin = 0.194;
    vmax = 0.868;

    [gcube, gtarget] = locate_cube_and_target(img, hmin, hmax, smin, smax, vmin, vmax);
end

function [bcube, btarget] = locate_blue_hsv(img)
    % Locate blue in HSV color space
    img = rgb2hsv(img);

    hmin = 0.563;  
    hmax = 0.729;
    smin = 0.100;
    smax = 0.900;
    vmin = 0.000;
    vmax = 0.612;

    [bcube, btarget] = locate_cube_and_target(img, hmin, hmax, smin, smax, vmin, vmax);
end

function [cube_centroid, target_centroid] = locate_cube_and_target(img, rmin, rmax, gmin, gmax, bmin, bmax)
    target_visible = 0;

    filter = (img(:, :, 1) >= rmin) & (img(:, :, 1) <= rmax) & ...
      (img(:, :, 2) >= gmin) & (img(:, :, 2) <= gmax) & ...
      (img(:, :, 3) >= bmin) & (img(:, :, 3) <= bmax);

    % Remove small areas from the binary image and fill holes in areas
    colored_area = bwareaopen(filter, 300);
    colored_area = imfill(colored_area, "holes");

    props = regionprops('table', colored_area, 'Centroid', 'Circularity', ...
        'MajorAxisLength','MinorAxisLength', 'Area', 'Eccentricity');


    cube_idx = find(min([props.MajorAxisLength] / [props.MinorAxisLength]));
    cube_centroid = [props.Centroid(cube_idx, :) 1];
    props([cube_idx], :) = [];

    
    if (height(props) >= 1)
        target_visible = 1;
        max_val = max(props.Circularity);
        target_idx = find(props.Circularity == max_val);
        target_centroid = [props.Centroid(target_idx, :) 1];
    else
        target_centroid = cube_centroid;
    end

    % Plotting for testing
    figure;
    imshow(colored_area);
    hold on;
    if (target_visible == 1)
        plot(target_centroid(1,1), target_centroid(1,2), "diamond", 'MarkerSize', 8, 'markerFaceColor', "red");
    end
    plot(cube_centroid(1,1), cube_centroid(1,2), "o", 'MarkerSize', 8, 'markerFaceColor', "red");
    hold off;
end

function [cyan_centroid, magenta_centroid] = locate_robot(img)
    % Locate cyan in RGB color space
    %rmin_c = 0;  
    %rmax_c = 100;
    %gmin_c = 50;
    %gmax_c = 150;
    %bmin_c = 100;
    %bmax_c = 255;

    % Locate cyan in HSV color space
    imghsv = rgb2hsv(img);
    hmin_c = 0.5;
    hmax_c = 0.61;
    smin_c = 0.217;
    smax_c = 1.0;
    vmin_c = 0.4;
    vmax_c = 1.0;

    cyan_centroid = locate_dot(imghsv, hmin_c, hmax_c, smin_c, smax_c, vmin_c, vmax_c);
    cyan_centroid = [cyan_centroid 1];

    % Locate magenta
    rmin_m = 70;  
    rmax_m = 255;
    gmin_m = 0;
    gmax_m = 55;
    bmin_m = 60;
    bmax_m = 255;

    magenta_centroid = locate_dot(img, rmin_m, rmax_m, gmin_m, gmax_m, bmin_m, bmax_m);
    magenta_centroid = [magenta_centroid 1];
end

function dot_centroid = locate_dot(img, rmin, rmax, gmin, gmax, bmin, bmax)
    filter = (img(:, :, 1) >= rmin) & (img(:, :, 1) <= rmax) & ...
      (img(:, :, 2) >= gmin) & (img(:, :, 2) <= gmax) & ...
      (img(:, :, 3) >= bmin) & (img(:, :, 3) <= bmax);

    colored_area = bwareaopen(filter, 30);
    colored_area = imfill(colored_area, "holes");

    props = regionprops('table', colored_area, 'Centroid', 'Circularity', ...
        'MajorAxisLength','MinorAxisLength', 'Area', 'Eccentricity');

    max_val = max(props.Circularity);
    dot_idx = find(props.Circularity == max_val);
    dot_centroid = props.Centroid(dot_idx, :);

    figure;
    imshow(colored_area);
    hold on;
    plot(dot_centroid(1,1), dot_centroid(1,2), "diamond", 'MarkerSize', 3, 'markerFaceColor', "red");
    hold off;
end

function robot = locate_robot_arrow(img)
    img = histeq(img);
    %figure;
    %imshow(img);

    %img = rgb2hsv(img);

    %hmin = 0.000; 
    %hmax = 1.000;
    %smin = 0.000;
    %smax = 0.071;
    %vmin = 0.880;
    %vmax = 1.000;

    %filter = (img(:, :, 1) >= hmin) & (img(:, :, 1) <= hmax) & ...
    %  (img(:, :, 2) >= smin) & (img(:, :, 2) <= smax) & ...
    %  (img(:, :, 3) >= vmin) & (img(:, :, 3) <= vmax);

    pmin = 99;
    pmax = 100;

    rmin = prctile(img(:, :, 1), pmin); 
    rmax = prctile(img(:, :, 1), pmax);
    gmin = prctile(img(:, :, 2), pmin);
    gmax = prctile(img(:, :, 2), pmax);
    bmin = prctile(img(:, :, 3), pmin);
    bmax = prctile(img(:, :, 3), pmax);

    filter = (img(:, :, 1) >= rmin) & (img(:, :, 1) <= rmax) & ...
      (img(:, :, 2) >= gmin) & (img(:, :, 2) <= gmax) & ...
      (img(:, :, 3) >= bmin) & (img(:, :, 3) <= bmax);

    colored_area = bwareaopen(filter, 50);
    colored_area = imfill(colored_area, "holes");

     labeled_img = bwlabel(colored_area);

    props = regionprops('table', labeled_img, 'Centroid', 'Circularity', ...
        'MajorAxisLength','MinorAxisLength', 'Area', 'BoundingBox');

    %max_val = max(props.Circularity);
    %dot_idx = find(props.Circularity == max_val);
    %dot_centroid = props.Centroid(dot_idx, :);

    arrow = [];
    indices = [];
    figure;
    imshow(colored_area);
    hold on;
    for i = 1:size(props)
        if (props.Circularity(i, :) > 0.8)
            dot_centroid = props.Centroid(i, :);
            %plot(dot_centroid(1,1), dot_centroid(1,2), "diamond", 'MarkerSize', 3, 'markerFaceColor', "red");
            arrow = [arrow,transpose(dot_centroid)];
            indices = [indices, i];
        end
    end
    display(arrow);
    arrow = filter_arrow(arrow, labeled_img, indices);
    for i = 1:8
        %plot(arrow(1, i), arrow(2, i), "diamond", 'MarkerSize', 3, 'markerFaceColor', "red");
    end

    hold off;

    robot = 1;  % Change this to sth meaningful
end

function arrow = filter_arrow(field, img, indices)
    res_arr = [];
    for i = 1:size(field,2)
        dist_ar = [];
        for j = 1:size(field,2)
            dist_x = abs(field(1,i)-field(1,j));
            dist_y = abs(field(2,i)-field(2,j));
            dist = sqrt(dist_x^2+dist_y^2);
            dist_ar = [dist_ar,dist]
        end
        res_arr = [res_arr,mean(dist_ar)];
    end
    while size(field,2) > 8
        [~,i] = max(res_arr);
        res_arr(i) = [];
        field(:,i) = [];
        indices(i) = [];
    end
    display(field);

    %labeled_img = bwlabel(img);
    img = ismember(img, indices);     % Filter out noise from the bw image
    dir=get_dir(field, img);
    arrow=field;

end

function dir = get_dir(arrow, img)
    box = [];
    [max_x,max_ix] = max(arrow(1,:))
    [min_x,min_ix] = min(arrow(1,:))
    [max_y,max_iy] = max(arrow(2,:))
    [min_y,min_iy] = min(arrow(2,:))

    box_mid = [min_x+(max_x - min_x)/2; min_y+(max_y-min_y)/2];
    display(arrow)
    display(mean(arrow(1)))
    anchor = [mean(arrow(1,:));mean(arrow(2,:))];

    dir=0;
    plot(max_x, max_y, "diamond", 'MarkerSize', 3, 'markerFaceColor', "yellow");
    plot(min_x, max_y, "diamond", 'MarkerSize', 3, 'markerFaceColor', "yellow");
    plot(max_x, min_y, "diamond", 'MarkerSize', 3, 'markerFaceColor', "yellow");
    plot(min_x, min_y, "diamond", 'MarkerSize', 3, 'markerFaceColor', "yellow");
    plot(box_mid(1), box_mid(2), "diamond", 'MarkerSize', 3, 'markerFaceColor', "yellow");
    plot(anchor(1), anchor(2), "diamond", 'MarkerSize', 3, 'markerFaceColor', "green");
    
   % vecs = zeros(9, 9, 2);
    arrow = [arrow,anchor];
    vecs = [];
    for i = 1:size(arrow,2)
        temp=[];
        for j = 1:size(arrow,2)
            lv = arrow(:,i) - arrow(:,j);
            temp = [temp,lv];
        end
        vecs=[vecs;temp];
    end
    disp(vecs)
    
    se = strel("disk", 10);
    closeimg = imclose(img, se);
    figure;
    %hold on;
    imshow(closeimg);
    %hold off;
    hold on;
    hold off;

    props = regionprops('table', closeimg, 'Orientation', ...
        'MajorAxisLength','MinorAxisLength', 'Area', 'Centroid');

    angle = props.Orientation;
    %if (angle < 0)
    %    angle = 180 + angle;
    %end
    display(angle);

    if ((angle > -45) && (angle < 0))
        % Check if the arrow head is on the top or bottom
        if (anchor(2) < box_mid(2))
            % On top
            angle
            display("top")
        else
            % On bottom
            angle = angle + 180
            display("bottom")
        end
    elseif ((angle > 0) && (angle < 45))
        % Check if the arrow head is on the top or bottom
        if (anchor(2) < box_mid(2))
            % On top
            angle = angle - 180
            display("top")
        else
            % On bottom
            angle
            display("bottom")
        end

    elseif ((angle > -90) && (angle < -45))
        % Check if the arrow head is on the left or right
        if (anchor(1) < box_mid(1))
            % On left
            angle
            display("left")
        else
            % On right
            angle = angle + 180
            display("right")
        end
    elseif ((angle > 45) && (angle < 90))
        % Check if the arrow head is on the left or right
        if (anchor(1) < box_mid(1))
            % On left
            angle = angle - 180
            display("left")
        else
            % On right
            angle = angle + 180
            display("right")
        end   
    end

end