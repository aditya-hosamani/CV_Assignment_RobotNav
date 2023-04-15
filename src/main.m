% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
%  Author: Aditya Hosamani (0585939)	        |
% ===============================================

% Capture images
% TBD

% Perform camera calibration
% Find the projection matrix using the list of images (imgs).
% squareSize (in mm) and boardSize (#corners) are optional, and can either
% be detected automatically from the images or provided explicitly by user
[projMatrix, camParams] = calibrate(imgs, squareSize, boardSize);

% Object detection

% Move the robot

