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
projMatrix = calibrate(calibImg);

img = imread("..\test_images\new_robot_cover\img5.png");
blocks = ["green","blue","red"];
x=move_block(blocks,img,projMatrix);

% Object detection

% Move the robot
