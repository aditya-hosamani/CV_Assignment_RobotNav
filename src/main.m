% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
% ===============================================

% Close open windows, clear workspace and console
close all; clear; clc;

% Capture images from the Kinect
% TBD

% Perform camera calibration
calibImg = imread("..\test_images\calibration\img1.png");
% points3d = 0.01 * points3d;     % convert cm to m
%projMatrix = calibrate(calibImg);

projMatrix = [0.0432    0.9514   -1.2587  574.0921;
    1.4682    0.7065   -0.6296  887.4907;
    0.0002   -0.0000   -0.0007    1.0000];

img = imread("..\test_images\calib_test\img2.png");
% % img = imread("test_images/images/img1.png");
blocks = ["green","blue","red"];
x=move_block(blocks,img,projMatrix)

% Object detection

% Move the robot
