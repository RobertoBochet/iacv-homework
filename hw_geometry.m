clear, close all;
img = imread('Image+-+Castello+di+Miramare.jpg');
img_g = rgb2gray(img);
imshow(img), hold on;

%% PI segments' points
s1 = Seg(HX([637 1267]), HX([1270 1177]));

s2 = Seg(HX([1410 1260]), HX([1606 1590]));

s3 = Seg(HX([1797 1585]), HX([2090 1717]));

s4 = Seg(HX([2173 1808]), HX([2690 1844]));

s5 = Seg(HX([2734 1847]), HX([2858 1697]));

s6 = Seg(HX([2974 1656]), HX([3944 1727]));

%% Draw plane PI
s1.draw;
s2.draw;
s3.draw;
s4.draw;
s5.draw;
s6.draw;

%% Add parallel lines to PI
s7 = Seg(HX([1831 2305]), HX([2127 2390]));
s8 = Seg(HX([3085 2665]), HX([2911 2750]));
s9 = Seg(HX([1260 2347]), HX([1508 2526]));

%% Calculate vanish point
v37 = s3.line * s7.line;
v58 = s5.line * s8.line;
v29 = s2.line * s9.line;

%% Draw vanish lines
v37|s3.P(1);
v37|s7.P(1);

v58|s5.P(2);
v58|s8.P(1);

v29|s2.P(1);
v29|s9.P(1);

%% Compute the line at the infinity
v_inf = [v37.X.'; v58.X.'; v29.X.'];

% we looking for the solution for l_inf that minimizing ||v_inf * l_inf||

[~, ~, v] = svd(v_inf);

l_inf = HX(v(1:end-1,end).');

l_inf.draw_line();

%% Compute the affine rectification matrix to send l_inf to its canonical position
h_aff = [1 0 0; 0 1 0; l_inf.X.'];

%% Show the rectificated image
imshow(imwarp(img_g, projective2d(h_aff.')));

%%
%J = imwarpLinear(rgb2gray(img), h_inf, [1, 1, 5000, 4000]);
%imshow(J)