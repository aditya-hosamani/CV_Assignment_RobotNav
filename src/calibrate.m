% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
%  Author: Aditya Hosamani (0585939)	        |
% ===============================================

function [projMat, camParams] = calibrate(imgs, squareSize, boardSize)
    % CALIBRATE Calibrate the camera using images of a scene.
    % Arguments:
    %           imgs - An array of images to be used for calibration
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
    
    % Generate the world coordinates of the corners of the chessboard pattern
    worldPoints = generateWorldPoints(boardSize, squareSize);
    
    % Create an empty matrix to store the image points
    imagePoints = [];
    
    % Load the images
    for imgIdx = size(imgs) % load 10 images
        img = imgs(imgIdx);
        
        % Detect the corners of the chessboard pattern in the image
        corners = harris_corner_detector(img, 10.0, 1000);
        % Display the corners on the original image
        %imshow(img);
        %hold on;
        %plot(corners(:, 1), corners(:, 2), 'r+', 'MarkerSize', 10);

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


function [worldPoints] = generateWorldPoints(boardSize, squareSize)
    % generateWorldPoints - Generates the world points of a checkerboard
    % Arguments:
    %    boardSize - The size of the board in terms of corners [rows, cols]
    %   squareSize - The size of each square block in mm
    %
    % Returns:
    %  worldPoints - 3D vector containing the generrated world points
    %

    % Create empty world point matrix
    worldPoints = zeros(prod(boardSize), 3);
    
    % Generate grid coordinates in the X-Y plane
    xGrid = repmat(1:boardSize(1), [1, boardSize(2)]);
    yGrid = reshape(repmat(1:boardSize(2), [boardSize(1), 1]), [1, prod(boardSize)]);
    xyGrid = [xGrid; yGrid];
    
    % Compute world coordinates of the grid corners
    worldPoints(:, 1:2) = squareSize * (xyGrid' - 1);
    
    % Set the Z-coordinate of the corners to zero
    worldPoints(:, 3) = 0;
end


function [corners] = harris_corner_detector(im, sigma, thresh)
    % HARRIS_CORNER_DETECTOR - Detect corners in an image using the Harris corner detector
    % Arguments:
    %          im - The input image
    %       sigma - The standard deviation of the Gaussian filter used to smooth the image (default = 1.5)
    %      thresh - The threshold value used to detect corners (default = 1e-6)
    %
    % Returns:
    %     corners - An Nx2 matrix of N corner points, where the first column is the x-coordinate and the second
    %                column is the y-coordinate
    %           H - The Harris response image
    %
    
    % Set default values
    if nargin < 2
        sigma = 1.5;
    end
    
    if nargin < 3
        thresh = 1e-6;
    end
    
    % Calculate image gradients
    Ix = conv2(im, [-1 0 1], 'same');
    Iy = conv2(im, [-1; 0; 1], 'same');
    
    % Compute products of derivatives at each pixel
    Ix2 = Ix .^ 2;
    Iy2 = Iy .^ 2;
    Ixy = Ix .* Iy;
    
    % Gaussian filter kernel
    k = ceil(sigma * 3) * 2 + 1;
    gauss = fspecial('gaussian', k, sigma);
    
    % Compute the sums of the products of derivatives at each pixel
    Sx2 = conv2(Ix2, gauss, 'same');
    Sy2 = conv2(Iy2, gauss, 'same');
    Sxy = conv2(Ixy, gauss, 'same');
    
    % Compute the Harris corner response function
    H = (Sx2 .* Sy2 - Sxy .^ 2) ./ (Sx2 + Sy2 + eps);
    
    % Find corners using non-maximum suppression
    Hmax = ordfilt2(H, 9, ones(3, 3));
    corners = (H == Hmax) & (H > thresh);
    
    % Get subscripts of the corner pixels
    [r, c] = find(corners);
    
    % Combine row and column subscripts into a single Nx2 matrix
    corners = [c, r];

    % Calculate pairwise distances between corner points
    N = size(corners, 1);
    dist = zeros(N, N);
    for i = 1:N
        for j = i+1:N
            dist(i,j) = norm(corners(i,:) - corners(j,:));
            dist(j,i) = dist(i,j);
        end
    end
    
    % Set the diagonal elements to infinity
    dist(1:size(dist, 1)+1:end) = inf;
    
    % Find neighboring corners based on the minimum distance
    [min_dist, ~] = min(dist, [], 2);
    neighbor_idx = find(min_dist < 60); % Change the threshold distance as needed
    
    % Retain only corners that have a neighboring corner
    corners = corners(neighbor_idx, :);

end



function [projM]  = estimateProjMat(points2d, points3d, normalizePts)
    % Function implementing DLT (Direct-Linear-Transform) to find and return
    % a suitable projection matrix to transform points3d to points2d
    % Arguments:
    %       points2d - 2D vector of image points
    %       points3d - 3D vector of world points
    %   normalizePts - Bool value checking if the points must be normalized
    %
    % Returns:
    %         projM - Projection matrix


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
    % normalize2D   -   Normalize 2D vector
    % Arguments:
    %    inPoints   -   Input points
    %
    % Returns:
    %            T  -   Transformation matrix for 2D vector normalization
    %  normPoints2d -   2D vector containing the generrated world points
    %

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
    % normalize3D   -   Normalize 3D vector
    % Arguments:
    %    inPoints   -   Input points
    %
    % Returns:
    %            T  -   Transformation matrix for 3D vector normalization
    %  normPoints3d -   3D vector containing the generrated world points
    %

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
    params.t = -R*C;
end
