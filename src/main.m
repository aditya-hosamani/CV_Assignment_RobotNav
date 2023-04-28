% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
% ===============================================

% Close open windows, clear workspace and console
close all; clear; clc;

% Capture images from the Kinect
% TBD

% Perform camera calibration
calibImg = imread("..\test_images\calibration\img4.png");
%projMatrix = calibrate(calibImg);
projMatrix = [1.48699200398622  -0.485026261014436   -0.634347291116932  887.059327916758;
    0.0684411527796349    0.459803451092746   -1.26108213837899  550.944988486190;
    0.000249883270920853   -0.000756377272536275   -0.000795545640202916    1.0000]

% save projMatrix projMatrix;

%load projMatrix.mat

img = imread("..\test_images\new_robot_cover\img21.png");

blocks = ["green","blue","red"];
x=move_block("red",img,projMatrix);

% Object detection

% Move the robot

% robot = [1540; 700];
% redCube = [1106; 595];
% % greenCube = [];
% blueCube = [1347; 440];
% redCircle = [770; 329];
% greenCircle = [920; 320];
% blueCircle = [1058; 315];
% 
% imgPoints = [robot, redCube, blueCube, redCircle, greenCircle, blueCircle];
% worldPoints = find3D(projMatrix, imgPoints);
% % Convert world-points to cm
% worldPoints = worldPoints/10;
% 
% % Distance between robot and red cube
% robotWorld = worldPoints(:,1);
% redCubeWorld = worldPoints(:,2);
% distance = norm(robotWorld - redCubeWorld);
% text = sprintf("Distance between robot and red cube = %d cm", round(distance));
% disp(text);
% 
% % Distance between robot and blue cube
% robotWorld = worldPoints(:,1);
% blueCubeWorld = worldPoints(:,3);
% distance = norm(robotWorld - blueCubeWorld);
% text = sprintf("Distance between robot and blue cube = %d cm", round(distance));
% disp(text);
% 
% % Distance between red, green and blue circles
% redCircleWorld = worldPoints(:,4);
% greenCircleWorld = worldPoints(:,5);
% blueCircleWorld = worldPoints(:,6);
% distance = norm(redCircleWorld - greenCircleWorld);
% text = sprintf("Distance between red and green circles = %d cm", round(distance));
% disp(text);
% 
% distance = norm(blueCircleWorld - greenCircleWorld);
% text = sprintf("Distance between blue and green circles = %d cm", round(distance));
% disp(text);



%% Auxiliary functions
% Reproject 2D to 3D points
function points3D = find3D(projM, points2D)
    % find3D     :   Find the 3D world coordinates from 2D image points
    % Arguments:
    %      projM - 3x4 Projection matrix
    %   points2D - 2D image points (2xM)
    %
    % Returns:
    %   points3D - 3D world coordinates (3xM)

    % Convert the given 2D point to its homogenous form
    n = size(points2D, 2);
    points3D = [points2D; ones(1, n)];

    % Compute the SVD of the projection matrix
    [U,S,V] = svd(projM);

    % Compute the pseudoinverse of the projection matrix P using the SVD
    P_hat = V * pinv(S) * U';

    % Compute the homogeneous 3D point in world coordinates
    for ptIdx = 1:n
        tempH = P_hat * points3D(:,ptIdx);
        % Normalize the 3D point
        tempWorld = tempH/tempH(4);
        points3D(:,ptIdx) = tempWorld(1:3);
    end

end

