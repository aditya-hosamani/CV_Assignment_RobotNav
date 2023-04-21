% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
%  Author: Aditya Hosamani (0585939)	        |
% ===============================================

% Close open windows, clear workspace and console
close all; clear; clc;

% Capture images from the Kinect
% TBD

% Perform camera calibration
% Read calibration image
calibImg = imread("test_images\calibration\img3.png");
% Number of squares along rows and columns
boardSize = [6,9];
% Size of each square in mm
squareSize = 45;
% [projMatrix, camParams] = calibrate(calibImg, squareSize, boardSize);

% Temporary calibration using MATLAB in-built
calibPath = "test_images\calibration";
[camParams, projMatrix]= calibrateMatLocal(calibPath);



%cameraParams = dltCalibration(calibImg, squareSize);

img = imread("test_images\new_robot_cover\imgx.png");
% img = imread("test_images/images/img1.png");
blocks = ["green","blue","red"];
x=move_block(blocks,img,projMatrix,camParams);

% Object detection

% Move the robot


%% Auxiliary functions
function [params, P] = calibrateMatLocal(calibPath)
    fileList = dir(calibPath);
    % Filter the list to only include image files
    imageFileNames = {};
    for i = 1:length(fileList)
        [~, fileName, ext] = fileparts(fileList(i).name);
        if strcmpi(ext, '.png')
            filePath = fullfile(calibPath, fileList(i).name);
            imageFileNames{end+1,1} = char(imageDatastore(filePath).Files);
        end
    end
    
    % Detect calibration pattern.
    [imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);
    
    % Generate world coordinates of the corners of the squares.
    squareSize = 45; % millimeters
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    % Calibrate the camera.
    imageSize = [1080, 1920];
    [params, ~, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
                                         ImageSize=imageSize);
    display(imagePoints)
    %intrinsic = params.Intrinsics;
    %extrinsic = estimateExtrinsics(imagePoints, worldPoints, intrinsic);
    [camProjection,~]=estimateCameraProjection(imagePoints,worldPoints)
    P = camProjection
end

