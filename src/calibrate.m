% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
%  Author: Aditya Hosamani (0585939)	        |
% ===============================================

function [projMat, camParams] = calibrate(imgs, squareSize, boardSize)
% CALIBRATE Calibrate the camera using images of a scene.
%    Inputs:
%       imgs: an array of images to be used for calibration
%       squareSize: the size (mm) of the edge of each square of checkerboard
%       boardSize: the number of corners in chessboard pattern [rows, cols]
%
%    Output:
%        projMat: The projection matrix of the camera
%        camParams: The intrinsic and extrinsic camera parameters

    % Define the size of the chessboard
    if nargin < 3
        squareSize = 25; % in millimeters
        boardSize = [6, 9]; % number of corners in the chessboard pattern
    end
    
    % Generate the world coordinates of the corners of the chessboard pattern
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    % Create an empty matrix to store the image points
    imagePoints = [];
    
    % Load the images
    for imgIdx = size(imgs) % load 10 images
        img = imgs(imgIdx);
        
        % Detect the corners of the chessboard pattern in the image
        [corners, isFound] = findChessboardCorners(img, boardSize);
        
        % If the corners are found, add them to the imagePoints matrix
        if isFound
            imagePoints = [imagePoints; corners];
        end
    end
    
    % Calculate the intrinsic and extrinsic camera parameters
    normalizePts = True;
    projMat = estimateProjMat(imagePoints, worldPoints, normalizePts);
    camParams = QR_decomp(projMatrix);
end


function [projM]  = estimateProjMat(points2d, points3d, normalizePts)
% Function implementing DLT (Direct-Linear-Transform) to find and return
% a suitable projection matrix to transform points3d to points2d

    % Convert points to homogenous form
    nPoints = size(points3d,2);
    points2d = [points2d; ones(1,nPoints)];
    points3d = [points3d; ones(1,nPoints)];
    
    % Normalize the points
    if normalizePts == true
        [T_2d, normPoints2d] = normalize2D(points2d);
        points2d = normPoints2d;
        [T_3d, normPoints3d] = normalize3D(points3d);
        points3d = normPoints3d;
    end

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
     if normalizePts == true
         projM = inv(T_2d) * projM * T_3d;
         projM = projM/ projM(3,4);
     end

end


function [T, normPoints2d] = normalize2D(inPoints)
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
    T = Ts*Tt;
    normPoints2d = Ts * inPoints;
end


function [T, normPoints3d] = normalize3D(inPoints)
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
    T = Ts*Tt;
    normPoints3d = Ts * inPoints;
end


function [params] = QR_decomp(projM)
% Function implementing QR decomposition to find the intrinsic and
% extrinsic camera parameters from the projection matrix M
    H = projM(:,1:3);
    h = projM(:,end);

    [R_trans, K_inv] = qr(inv(H));

    params.R = R_trans';
    params.K = inv(K_inv);

    % Calculating camera position center
    C = -inv(H)*h;
    params.t = -R*C;
end
