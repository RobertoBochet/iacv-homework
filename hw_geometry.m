%% ######## G1 2D reconstruction ########
clear, close all;
img = imread('Image+-+Castello+di+Miramare.jpg');
img_g = rgb2gray(img);
imshow(img), hold on;

%% PI segments' points
% defines the given line segments of plane PI exploiting class Seg and HX
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
% defines additional line segments paraller to the plane PI
s7 = Seg(HX([1831 2305]), HX([2127 2390]));
s8 = Seg(HX([3085 2665]), HX([2911 2750]));
s9 = Seg(HX([1260 2347]), HX([1508 2526]));

%% Calculate vanish points
v37 = s3.line * s7.line;
v58 = s5.line * s8.line;
v29 = s2.line * s9.line;

v37.draw_point;
v58.draw_point;
v29.draw_point;

%% Draw vanish lines
lv3 = Seg(v37, s3.P(1));
lv7 = Seg(v37, s7.P(1));
lv5 = Seg(v58, s5.P(2));
lv8 = Seg(v58, s8.P(1));
lv2 = Seg(v29, s2.P(1));
lv9 = Seg(v29, s9.P(1));

lv3.draw;
lv7.draw;
lv5.draw;
lv8.draw;
lv2.draw;
lv9.draw;

%% Compute the line at the infinity
% prepares the matrix v_inf for LST
% to find the best approximation of infinity line l_inf
v_inf = [v37.X.'; v58.X.'; v29.X.'];

% looks for the solution for l_inf that minimizing ||v_inf * l_inf||
[~, ~, V] = svd(v_inf);

l_inf = HX(V(1:end-1,end).');

l_inf.draw_line();

%% Compute the affine rectification matrix to send l_inf to its canonical position
H_p = [1 0 0; 0 1 0; l_inf.X.'];

%% Show the rectificated image
img_a = imwarp(img_g, projective2d(H_p.'));
figure, imshow(img_a, "InitialMagnification", "fit"), hold on;

%% Affine rectify the ortogonal segments
% This procedure is equivalent to rectify directly the corresponding lines
s1_a = H_p * s1;
s2_a = H_p * s2;
s3_a = H_p * s3;
s4_a = H_p * s4;
s5_a = H_p * s5;
s6_a = H_p * s6;

%% Draw the affine rectificated plane PI
s1_a.draw;
s2_a.draw;
s3_a.draw;
s4_a.draw;
s5_a.draw;
s6_a.draw;

%% Compute the matrix to find infinite line conic via LSM
% all the 3 couples of orthogonal lines are used
% (istead of only 2) to reduce error
l1 = s1_a.line.X;
l2 = s2_a.line.X;
l4 = s4_a.line.X;
l5 = s5_a.line.X;
l6 = s6_a.line.X;

C_d = [[l1(1) * l2(1), l1(1) * l2(2) + l1(2) * l2(1), l1(2) * l2(2)];
	   [l4(1) * l5(1), l4(1) * l5(2) + l4(2) * l5(1), l4(2) * l5(2)];
	   [l5(1) * l6(1), l5(1) * l6(2) + l5(2) * l6(1), l5(2) * l6(2)]];

%% Searching the infinite line conic in affine rectificated image
% looks for the solution for s that minimizing ||C_d * s|| to find C_inf
[~, ~, V] = svd(C_d);

s = V(:,end);

C_inf = [[s(1) s(2) 0]; [s(2) s(3) 0]; [0 0 0]];

%% Looking for affine trasformation to map c_inf to its canonical position
[U,D,V] = svd(C_inf);

H_a = U*sqrt(D)*V' + diag([0 0 1]);
% invert H_a
H_a = eye(3) / H_a;

% computes the complessive homography
H = H_a * H_p;

%% Show the rectificated image
t_a = affine2d(H_a.');

sameAsInput = affineOutputView(size(img_a),t_a,'BoundsStyle','SameAsInput');
img_ap = imwarp(img_a, t_a,'OutputView',sameAsInput);

figure, imshow(img_ap, "InitialMagnification", "fit"), hold on;

%% Draw PI on the metric image
s1_m = H * s1;
s2_m = H * s2;
s3_m = H * s3;
s4_m = H * s4;
s5_m = H * s5;
s6_m = H * s6;

s1_m.draw;
s2_m.draw;
s3_m.draw;
s4_m.draw;
s5_m.draw;
s6_m.draw;

%% Compute the translation to put the reference frame at the s1 s2 intersection

o = s1_m.line * s2_m.line;

H_t = [eye(2), -o.cart;zeros(2,1)' 1];

H = H_t * H_a * H_p;

%% Draw PI
s1_t = H * s1;
s2_t = H * s2;
s3_t = H * s3;
s4_t = H * s4;
s5_t = H * s5;
s6_t = H * s6;

figure, hold on, daspect([1 1 1]);
s1_t.draw;
s2_t.draw;
s3_t.draw;
s4_t.draw;
s5_t.draw;
s6_t.draw;

%% Rotate the frame
% put the segment s2 on the y-axis
rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1];

p = s2_t.P(1).cart();
theta = atan2(p(2), p(1));

H_r = rotz(pi/2-theta);

H = H_r * H_t * H_a * H_p;

%% Draw PI
s1_r = H * s1;
s2_r = H * s2;
s3_r = H * s3;
s4_r = H * s4;
s5_r = H * s5;
s6_r = H * s6;

figure, hold on, daspect([1 1 1]);
s1_r.draw;
s2_r.draw;
s3_r.draw;
s4_r.draw;
s5_r.draw;
s6_r.draw;

%% Rescale the frame
% uses the real (approximal) size in meter of the line segment s4
% to rescale all the points
r_len = 5.9;
s4_len = Seg(s4_r.line * s5_r.line, s5_r.line * s6_r.line).length;

% computes the scale factor
s = r_len / s4_len;

% makes the scale matrix
H_s = diag([s s 1]);

% computes the new homography
H = H_s * H_r * H_t * H_a * H_p;

%% Draw PI rescaled
s1_s = H * s1;
s2_s = H * s2;
s3_s = H * s3;
s4_s = H * s4;
s5_s = H * s5;
s6_s = H * s6;

% computes the line segments for a close line 
s1_f = Seg(s1_s.P(1), s1_s & s2_s);
s2_f = Seg(s1_s & s2_s, s2_s & s3_s);
s3_f = Seg(s2_s & s3_s, s3_s & s4_s);
s4_f = Seg(s3_s & s4_s, s4_s & s5_s);
s5_f = Seg(s4_s & s5_s, s5_s & s6_s);
s6_f = Seg(s5_s & s6_s, s6_s.P(2));

figure, hold on, daspect([1 1 1]);
s1_f.draw;
s2_f.draw;
s3_f.draw;
s4_f.draw;
s5_f.draw;
s6_f.draw;