% ===============================================
%  Course: Computer Vision (BM40A901)	        |
%  Practical Assignment: Collecting Cubes		|
%  Author: Aditya Hosamani (0585939)	        |
% ===============================================

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
end

