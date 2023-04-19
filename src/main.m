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
[projMatrix, camParams] = calibrate(calibImg, squareSize, boardSize);

img = imread("test_images\new_robot_cover\imgx.png");
% img = imread("test_images/images/img1.png");
blocks = ["green","blue","red"];
x=move_block(blocks,img,projMatrix,camParams);

% Object detection

% Move the robot

