% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
%  Author: Aditya Hosamani (0585939)	        |
% ===============================================

function [projM] = calibrate(inputImg)
    % CALIBRATE     :   Calibrate the camera using a single input image
    % Arguments:
    %      inputImg - The calibration image
    %
    % Returns:
    %        projMat - The projection matrix of the camera

    figHandle = showCornerTemplate();

    points2d = get2dUserInput(inputImg);
    close(figHandle);   % close the template image once all 2D user points have been selected

    points3d = generateWorldPoints();

    %save points2dTest points2d;
    %save points3dTest points3d;
    
    %load points2d.mat;
    %load points3d.mat;
    %points2d = points2d';
    %points3d = points3d';
    
    projM = performDLT(points3d, points2d);

    % Define the residual function
    % Convert the given 3D point to its homogenous form
    n = size(points3d, 2);
    points3d = [points3d; ones(1, n)];
    
    display(projM);
    display(points2d);
    x = [projM(:); points2d(:)];
    residuals = @(x) optimize_calibration(x, points3d);
    
    % Use lsqnonlin to optimize the projection matrix
    options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');
    x = lsqnonlin(residuals, x, [], [], options);
    projM = reshape(x(1:12), [3 4]);
    points2d = reshape(x(13:end), [2 13]);
    

    % Verify projection matrix by displaying the projected points
    points3d = points3d(1:3,:);
    projPoints2d = project3d_2d(points3d, projM);
    f2 = figure;
    imshow(inputImg);
    hold on;
    plot(points2d(1,:), points2d(2,:), 'r+', 'MarkerSize', 10);
    plot(projPoints2d(1,:), projPoints2d(2,:), 'bo', 'MarkerSize', 10);

    % Display the reprojection error
    reprojError = vecnorm(points2d-projPoints2d, 2, 1);
    fprintf("Reprojection Error: %f\n", reprojError);
    
end

function residuals = optimize_calibration(x, points3d)
     projMatrix = reshape(x(1:12), [3 4]);
     points2d = reshape(x(13:end), [2 13]);
     residuals = sqrt(sum((pflat(projMatrix * points3d) - points2d).^2, 1));
 end

function X_eucl = pflat(X_homog)
% Convert homogeneous coordinates to Euclidean coordinates
% by dividing the first n elements of each row by the last element
n = size(X_homog, 1) - 1;
X_eucl = X_homog(1:n, :) ./ X_homog(end, :);
end


function figHandle = showCornerTemplate()
    %  SHOWCORNERTEMPLATE   :   Show template image to show the order of
    %  selecting corners for 2D image points
    %

    templateImg = imread("..\test_images\calibration\cornerTemplate.png");
    figHandle = figure();
    imshow(templateImg);
    title('Corner selection template')

end


function [points3d] = generateWorldPoints()
    % GENERATEWORLDPOINTS   :   Generate world points for the cube corners
    %
    % Returns:
    %        points3d       -   The 3D world points (3xM)

    points3d = [ [0,0,50]; [0,0,0]; [315,0,0]; [315,0,50]; [360,0,50]; ...
        [360,0,0]; [510,180,0]; [510,180,50]; [510,230,50]; [510,230,0];...
        [560,230,0]; [560,230,50]; [560,180,50]];
    points3d = points3d';

end



function [projM] = performDLT(points3d, points2d)
    % CALIBRATE     :   Find projection matrix using the 3D world points
    %                   and 2D image points
    % Arguments:
    %    points2d   -   2D image points (2xM)
    %    points3d   -   3D world points (3xM)
    %
    % Returns:
    %        projM  -   The projection matrix of the camera

    % Convert points to homogenous form
    nPoints = size(points3d,2);
    points2d = [points2d; ones(1,nPoints)];
    points3d = [points3d; ones(1,nPoints)];
    
    % Normalize the points
    [T_2d, normPoints2d] = normalize2D(points2d);
    points2d = normPoints2d;
    [T_3d, normPoints3d] = normalize3D(points3d);
    points3d = normPoints3d;

    % Compute A for Am = 0
    n = 12;
    A = zeros(2*nPoints, n);

    for i=1:nPoints
        u = points2d(1,i);
        v = points2d(2,i);
        w = points2d(3,i);

        x = points3d(:,i);

        Ai =    [ 0, 0, 0, 0,   -w * x',   v * x' ;
                w * x',   0, 0, 0, 0,  -u * x' ];

        A = [A; Ai];
    end

    % Use SVD to compute the projection matrix from A
    [~,~,V] = svd(A);
    h = V(:,n);
    h = h/h(n);
    projM = reshape(h,4,3)';

    % Denormalize the projection matrix
    projM = inv(T_2d) * projM * T_3d;
    projM = projM/ projM(3,4);

end


function imgPoints = get2dUserInput(inputImg)
    % get2dUserInput:   Get the 2D image points from user-clicks
    % Arguments:
    %      inputImg -   The calibration image
    %
    % Returns:
    %      imgPoints -  An array of 2xM image points

    calibPoints = 13;
    f1 = figure;
    B = imshow(inputImg);
    imgPoints = [];
    row = []; column = [];
    while 0<1
        pointsSelected = size(row,1);
        titleTxt = sprintf("%d Corners selected", pointsSelected);
        title(titleTxt);
        [x,y,b] = ginput(1);
        if isempty(b)
            break;
        elseif b==45        % ASCII code for '-' sign
            ax = axis; width=ax(2)-ax(1); height=ax(4)-ax(3);
            axis([x-width/2 x+width/2 y-height/2 y+height/2]);
            zoom(1/2);
        elseif b==43        % ASCII code for '+' sign
            ax = axis; width=ax(2)-ax(1); height=ax(4)-ax(3);
            axis([x-width/2 x+width/2 y-height/2 y+height/2]);
            zoom(2);    
        else
            row=[row;round(x)];
            column=[column;round(y)];
        end
        if(size(row,1) == calibPoints)
            break
        end
    end
    imgPoints = [row column];
    imgPoints = imgPoints';
    close(f1);

end


function [T_2D, normPoints2d] = normalize2D(inPoints)
    % NORMALIZE2D   :   Normalize 2D image points
    % Arguments:
    %    points2D   -   Image points
    %
    % Returns:
    %  normPoints2D -   2D vector containing the detected image points
    %          T_2D -   Transformation matrix for 2D vector normalization

    % initialize T
    Tt = eye(3);
    % calculate centroid -> translation component known
    t = [-mean(inPoints(1,:)), -mean(inPoints(2,:))];
    Tt(1:2,3) = t;
    
    % translate points
    inPoints = Tt*inPoints;

    % compute scaling -> scaling component of T known
    % avgdistance from origin = 1/m * sum(norm(x)), where m = # points
    % compute norm of each vector
    nx = sqrt(sum(inPoints(1:2,:).^2));
    % we want avgdist_from_O = s * (1/m) * sum(norm(x)) = sqrt(2) 
    % <-> s = sqrt(2) / (1/m) * sum(nx) = sqrt(2) / mean(nx)
    s = sqrt(2) / mean(nx);
    
    % construct T from t and s
    Ts = [s 0 0
         0 s 0 
         0 0 1];
    % translation followed by scaling
    T_2D = Ts*Tt;
    normPoints2d = Ts * inPoints;

end


function [T_3D, normPoints3d] = normalize3D(inPoints)
    % NORMALIZE3D   :   Normalize 3D image points
    % Arguments:
    %    inPoints   -   3D World points
    %
    % Returns:
    %  normPoints3d -   3D vector containing the detected image points
    %          T_3D -   Transformation matrix for 3D vector normalization

    % initialize T
    Tt = eye(4);
    % calculate centroid -> translation component known
    t = [-mean(inPoints(1,:)), -mean(inPoints(2,:)), -mean(inPoints(3,:))];
    Tt(1:3,4) = t;
    
    % translate points
    inPoints = Tt*inPoints;
    % compute scaling -> scaling component of T known
    
    % avgdistance from origin = 1/m * sum(norm(x)), where m = # points
    % compute norm of each vector
    nx = sqrt(sum(inPoints(1:3,:).^2));
    % we want avgdist_from_O = s * (1/m) * sum(norm(x)) = sqrt(2) 
    % <-> s = sqrt(2) / (1/m) * sum(nx) = sqrt(2) / mean(nx)
    s = sqrt(3) / mean(nx);
    
    % construct T from t and s
    Ts = [s 0 0 0
         0 s 0 0
         0 0 s 0
         0 0 0 1];
    % translation followed by scaling
    T_3D = Ts*Tt;
    normPoints3d = Ts * inPoints;

end


function [projPoints2d] = project3d_2d(points3d, projM)
    % PROJECT3D_2D  :   Find the 2D projections of 3D world points using the
    %                   projection matrix
    % Arguments:
    %    points3d   -   3D World points (3xM)
    %    projM      -   Projection matrix (3x4)
    %
    % Returns:
    %  projPoints2d -   2D projections of 3D world points

    nPoints = size(points3d,2);
    tmp = ones(nPoints,1)';
    projected_points = zeros(nPoints,3,1);
    %disp(size(projected_points));
    %disp(size(tmp));
    homogenous_points = [points3d; tmp];
    %disp(size(homogenous_points));
    projected_points =  projM * homogenous_points;
    %disp(size(projected_points));

    projPoints2d = zeros(2,size(points3d,2));

    projPoints2d(1,:) = round(projected_points(1,:,:) ./ projected_points(3,:,:));
    projPoints2d(2,:) = round(projected_points(2,:,:) ./ projected_points(3,:,:));

end
