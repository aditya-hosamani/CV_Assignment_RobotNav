% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
% ===============================================

% Close open windows, clear workspace and console
close all; clear; clc;

% Capture images from the Kinect
% TBD

% Perform camera calibration
calibImg = imread("..\test_images\calibration\img2.png");
projMatrix = calibrate(calibImg);
%projMatrix = [1.3311   -0.6213   -0.7263  821.6881;
%    0.0361    0.4096   -1.2731  518.8264;
%    0.0001   -0.0008   -0.0009    1.0000]

img = imread("..\test_images\new_robot_cover\img7.png");
blocks = ["green","blue","red"];
x=move_block(blocks,img,projMatrix);

% Object detection

% Move the robot
