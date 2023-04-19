% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
%  Author: Aditya Hosamani (0585939)	        |
% ===============================================

function [projMat, camParams] = calibrate(img, squareSize, boardSize)
    % CALIBRATE Calibrate the camera using images of a scene.
    % Arguments:
    %           img  - An array of images to be used for calibration
    %     squareSize - The size (mm) of the edge of each square of checkerboard
    %      boardSize - The number of corners in chessboard pattern [rows, cols]
    %
    % Returns:
    %        projMat - The projection matrix of the camera
    %      camParams - The intrinsic and extrinsic camera parameters

    % Define the size of the chessboard
    if nargin < 3
        squareSize = 25; % in millimeters
        boardSize = [6,9]; % number of corners in the chessboard pattern
    end

    
    % smooth the image to reduce noise
    smoothImg = imgaussfilt(img, 0.3);

    % convert to grayscale
    gray = rgb2gray(smoothImg);

    maskedImg = imsharpen(imageMasker(gray));
    %imwrite(maskedImg, "maskedImg.jpg")
    
    % Generate the world coordinates of the corners of the chessboard pattern
    worldPoints = generateWorldPoints(boardSize, squareSize);
    
    % Create an empty matrix to store the image points
    imagePoints = [];
    
    % Detect the corners of the chessboard pattern in the image
    corners = harris_corner_detector(maskedImg, 15.0, 1100);
    
    % ---------------------The below is done for img3.png to counter the corner ordering ----------------
    temp = corners(35,:);
    corners(35,:) = corners(36,:);
    corners(36,:) = temp;
    % ----------------------------------------------------------------------------------------------------

    %save corners.mat corners;
    % Display the corners on the original image
    %imshow(img);
    %hold on;
    %plot(corners(:, 1), corners(:, 2), 'r+', 'MarkerSize', 10);
    %title("Detected " + size(corners,1) + " corners");

    % If all checkerboard corners are not found, exit the function
    isFound = false;
    nCorners = (boardSize(1)-1) * (boardSize(2)-1);
    if size(corners,1) ~= nCorners
        disp("Failed to detect the required checkerboard corners only. Please provide a new image");
        projMat = -1;
        camParams = -1;
    end

    % If the corners are found, add them to the imagePoints matrix
    imagePoints = [imagePoints; corners];
    %save imagePoints imagePoints;
    
    % Calculate the intrinsic and extrinsic camera parameters
    % Reshape the points to 2xn and 3xn
    imagePoints = imagePoints';
    worldPoints = worldPoints';
    save worldPoints worldPoints;
    save imagePoints imagePoints;
    projMat = estimateProjMat(imagePoints, worldPoints);
    camParams = QR_decomp(projMat);
end


function mImg = imageMasker(inputImg)
    % Masks the input image such that the background clutter is minimized
    % leaving better visiblity for the checkerboard
    %
    %   Arguments:
    %       inputImg    - Grayscale input image
    %
    %   Returns:
    %       mImg        - Grayscale masked image
    %
    
    % Assuming the calibration target would be kept on the table, the
    % table region is manually marked from the sample images.
    % ROI boundaries: TL, TR, BR, BL
    %maskROI = [[0,300], [1500,300], [1920,1080], [0,1080]];
    mImg = inputImg;
    % make the top black
    mImg(1:400, 1:1920) = 0;
    % make the left-edge black
    mImg(400:800, 1:400) = 0;
end


function [worldPoints] = generateWorldPoints(boardSize, squareSize)
    %GENERATEWORLDPOINTS Generates 3D world points for a checkerboard pattern
    %
    %   Arguments:
    %       boardSize  - [rows, cols] size of the checkerboard pattern
    %       squareSize - scalar size of each checkerboard square in mm
    %
    %   Returns:
    %       worldPoints - (rows * cols, 3) matrix of 3D world points
    %
    
    % Initialize worldPoints matrix
    numPoints = (boardSize(1)-1) * (boardSize(2)-1);
    worldPoints = zeros(numPoints, 3);
    
    % Generate 3D world points
    [X, Y] = meshgrid(0:(boardSize(2)-2), 0:(boardSize(1)-2));
    worldPoints(:, 1) = X(:) * squareSize;
    worldPoints(:, 2) = Y(:) * squareSize;
    worldPoints(:, 3) = 0;
end


function [projM]  = estimateProjMat(points2d, points3d)
    % Function implementing DLT (Direct-Linear-Transform) to find and return
    % a suitable projection matrix to transform points3d to points2d
    % Arguments:
    %       points2d - 2D vector of image points
    %       points3d - 3D vector of world points
    %
    % Returns:
    %         projM - Projection matrix


    % Convert points to homogenous form
    nPoints = size(points3d,2);
    points2d = [points2d; ones(1,nPoints)];
    points3d = [points3d; ones(1,nPoints)];
    
    % Normalize the points
    [normPoints2D, T_2D, normPoints3D, T_3D] = normalize(points2d, points3d);
    %save normPoints2D normPoints2D
    %save normPoints3D normPoints3D

    % Compute A for Am = 0
    n = 12;
    A = zeros(2*nPoints, n);

    for i=1:nPoints
        u = normPoints2D(1,i);
        v = normPoints2D(2,i);
        w = normPoints2D(3,i);

        x = normPoints3D(:,i);

        Ai =    [ 0, 0, 0, 0,   -w * x',   v * x' ;
                w * x',   0, 0, 0, 0,  -u * x' ];

        A = [A; Ai];
    end


    % Use SVD to compute the projection matrix from A
    [~,~,V] = svd(A);
    m = V(:,end);
    h = m/m(end);
    projM = reshape(h, [4,3])';

    % Denormalize the projection matrix
    projM = inv(T_2D) * projM * T_3D;
    projM = projM/ projM(3,4);

end


function [normPoints2D, T_2D, normPoints3D, T_3D] = normalize(points2D, points3D)
    % normalize     -   Normalize 2D and 3D image and world points
    % Arguments:
    %    points2D   -   Image points (2xN)
    %    points3D   -   World points (3xN)
    %
    % Returns:
    %  normPoints2D -   2D vector containing the detected image points
    %          T_2D -   Transformation matrix for 2D vector normalization
    %  normPoints3D -   3D vector containing the generrated world points
    %          T_3D -   Transformation matrix for 3D vector normalization

    n = size(points2D, 2);
    % normalize image points
    u = points2D(1, :);
    v = points2D(2, :);
    dist = mean(sqrt((u - mean(u)) .* (u - mean(u)) + (v - mean(v)) .* (v - mean(v))));
    T_2D = [sqrt(2) / dist  0            -sqrt(2) * mean(u) / dist;
         0            sqrt(2) / dist  -sqrt(2) * mean(v) / dist;
         0            0            1];
    normPoints2D = [u; v; ones(1, n)];
    normPoints2D = T_2D * normPoints2D;
    
    % normalize world points
    X = points3D(1, :);
    Y = points3D(2, :);
    Z = points3D(3, :);
    dist = mean(sqrt((X - mean(X)) .* (X - mean(X)) + (Y - mean(Y)) .* (Y - mean(Y)) + (Z - mean(Z)) .* (Z - mean(Z))));
    T_3D = [sqrt(3) / dist  0               0       -sqrt(3) * mean(X) / dist;
                0       sqrt(3) / dist      0       -sqrt(3) * mean(Y) / dist;
                0           0       sqrt(3) / dist  -sqrt(3) * mean(Z) / dist;
                0           0               0                1];
    normPoints3D = [X; Y; Z; ones(1, n)];
    normPoints3D = T_3D * normPoints3D;

end


function [params] = QR_decomp(projM)
    % QR_decomp  -   QR decomposition to find the intrinsic and
    % extrinsic camera parameters from the projection matrix M
    % Arguments:
    %    projM   -   Projection matrix of size 3x4
    %
    % Returns:
    %    params  -   A struct containing the intrinsic and extrinsic camera
    %    parameters
    %

    H = projM(:,1:3);
    h = projM(:,end);

    [R_trans, K_inv] = qr(inv(H));

    params.R = R_trans';
    params.K = inv(K_inv);

    % Calculating camera position center
    C = -inv(H)*h;
    params.t = -params.R*C;
end

