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
projMatrix = calibrate(calibImg);


img = imread("..\test_images\new_robot_cover\img5.png");
% % img = imread("test_images/images/img1.png");
blocks = ["green","blue","red"];
x=move_block(blocks,img,projMatrix);

% Object detection

% Move the robot
