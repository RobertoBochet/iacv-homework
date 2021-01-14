clear;
img = imread('Image+-+Castello+di+Miramare.jpg');
img = im2double(img);
%imshow(img);

%% Image crop
% Crops the image to exclude part of the sky which does not contain useful informations.
img = img(900:end,:,:);
%imshow(img)

%% Sky mask
% Creates a mask excluding the useless sky color based.
% The mask is improved with dilate function.
img_hsv = rgb2hsv(img);

min_v = 0;
max_v = 0.80;

sky_mask = (min_v <= img_hsv(:,:,3)) & (img_hsv(:,:,3) <= max_v);
sky_mask = imdilate(sky_mask, strel('square',30));

img_masked = bsxfun(@times, img, cast(sky_mask, 'like', img));

%imshow(img_masked)

%% 
%{
shadow_lab = rgb2lab(img_masked);
max_luminosity = 100;
L = shadow_lab(:,:,1)/max_luminosity;

shadow_imadjust = shadow_lab;
shadow_imadjust(:,:,1) = imadjust(L)*max_luminosity;
shadow_imadjust = lab2rgb(shadow_imadjust);
%shadow_imadjust = rgb2gray(shadow_imadjust);

shadow_histeq = shadow_lab;
shadow_histeq(:,:,1) = histeq(L)*max_luminosity;
shadow_histeq = lab2rgb(shadow_histeq);
%shadow_histeq = rgb2gray(shadow_histeq);

shadow_adapthisteq = shadow_lab;
shadow_adapthisteq(:,:,1) = adapthisteq(L)*max_luminosity;
shadow_adapthisteq = lab2rgb(shadow_adapthisteq);
%shadow_adapthisteq = rgb2gray(shadow_adapthisteq);


%shadow_imadjust = locallapfilt(shadow_imadjust, 0.1, 0.2, 'NumIntensityLevels', 50);
%shadow_histeq = locallapfilt(shadow_histeq, 0.1, 0.2, 'NumIntensityLevels', 50);
%shadow_adapthisteq = locallapfilt(shadow_adapthisteq, 0.1, 0.2, 'NumIntensityLevels', 50);

imshow([img_masked,shadow_imadjust;shadow_histeq,shadow_adapthisteq])
%}

%% Edges detection
img_edge = img_masked;
img_edge = rgb2lab(img_edge);
max_luminosity = 100;
L = img_edge(:,:,1)/max_luminosity;
shadow_adapthisteq = img_edge;
shadow_adapthisteq(:,:,1) = adapthisteq(img_edge)*max_luminosity;
img_edge = lab2rgb(shadow_adapthisteq);
img_edge = imgaussfilt(img_edge, 4);
img_edge = rgb2gray(img_edge);
edges = edge(img_edge,'canny',[0.01,0.15]);

%imshow(edges);

%% Vertical line
[H,T,R] = hough(edges, "Theta", (-13:13));%, 'RhoResolution', 0.1);
P = houghpeaks(H, 200,'threshold', ceil(0.3*max(H(:))), "NHoodSize", [3,3]);

lines = houghlines(edges, T, R, P,'FillGap', 70,'MinLength', 500);

draw_lines(img, lines)

%% Horizontal line
[H,T,R] = hough(edges, "Theta", [(-90:-75) (75:89)]);%, 'RhoResolution', 0.1);
P = houghpeaks(H, 200,'threshold',ceil(0.3*max(H(:))), "NHoodSize", [3,3]);

lines = houghlines(edges, T, R, P,'FillGap', 50,'MinLength', 300);

draw_lines(img, lines)

%% Features detection
img_corners = img;
img_corners = rgb2gray(img_corners);
img_corners = roifilt2(img_corners, sky_mask, 'adapthisteq');

corners = detectSURFFeatures(img_corners);
imshow(img), hold on;
plot(corners.selectStrongest(100));

%%

























