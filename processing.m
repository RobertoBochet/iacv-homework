%% ######## F1 Feature extraction ########
clear, close all, clear HX;
img = imread('input.jpg');
% converts image's values in double notation
img = im2double(img);

%% Image crop
% crops the image to exclude part of the sky which does not contain useful information
img = img(900:end,:,:);

%% Compute the sky mask
% creates a mask excluding the useless sky color based.
% changes the color space to hsv
img_hsv = rgb2hsv(img);

min_v = 0;
max_v = 0.80;

% selects a range on the channel V
sky_mask = (min_v <= img_hsv(:,:,3)) & (img_hsv(:,:,3) <= max_v);

% the mask is improved with dilate function.
sky_mask = imdilate(sky_mask, strel('square',30));

% applies the mask on the image
img_masked = bsxfun(@times, img, cast(sky_mask, 'like', img));

figure; imshow(img_masked);

%% Histogram equalization

img_edge = img;
% converts the 3 channels image to one channel image
img_edge = rgb2gray(img_edge);
% applies the adaptive histogram equalization only on the castle (excluding the sky)
img_edge = roifilt2(img_edge, sky_mask, 'adapthisteq');

figure; imshow(img_edge);

%% Edges detection
% computes the rescaled image size
k = 0.4;
rescaled_size = k*size(img,1,2);

% rescales the image
img_edge = imresize(img_edge, rescaled_size, 'bilinear');

% applies the canny algorithm
edges = edge(img_edge, 'canny', [0.1, 0.2], 3);

figure; imshow(edges);

%% Detecting lines
% applies the Hough transformation
[H,T,R] = hough(edges, "Theta", (-90:1:89), 'RhoResolution', 1);
% selects the peaks in the parameters plane
P = houghpeaks(H, 300,'threshold', ceil(0.1*max(H(:))), "NHoodSize", [15,15]);

% searches the segments lines in the image given the peak lines
lines = houghlines(edges, T, R, P,'FillGap', 8,'MinLength', 25);

figure; imshow(img_edge), hold on;
draw_lines(lines);

%% Features detection

img_corners = img;
% converts the image to gray scale
img_corners = rgb2gray(img_corners);
% applies the histogram equalization
img_corners = roifilt2(img_corners, sky_mask, 'adapthisteq');

% applies the SURF algorithm
corners = detectSURFFeatures(img_corners);

figure; imshow(img_corners), hold on;

% plots the strongest features
plot(corners.selectStrongest(500));
