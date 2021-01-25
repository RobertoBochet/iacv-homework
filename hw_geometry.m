%% ######## G1 2D reconstruction ########
clear, close all, clear HX;
img = imread('input.jpg');
img_g = rgb2gray(img);
imshow(img), hold on;

HX.drawing_limits([
				[-3*size(img,1) 3*size(img,1)];
				[-3*size(img,2) 3*size(img,2)]]);

set(0, 'DefaultLineLineWidth', 2);

%% PI segments' points
% defines the given line segments of plane PI exploiting class Seg and HX
s1 = Seg(HX([637 1267]), HX([1270 1177]));
s2 = Seg(HX([1410 1260]), HX([1607 1581]));
s3 = Seg(HX([1797 1585]), HX([2090 1717]));
s4 = Seg(HX([2173 1808]), HX([2687 1842]));
s5 = Seg(HX([2740 1833]), HX([2854 1692]));
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
% for the castle's facades 2,3 and 5 
sg2 = SegGroup([
	1391.8 1831.1  1532.7 1994.1;
	1396.5 1597.2  1538.6 1788.7;
	1259.6 2344.5  1498.5 2516.0;
	1356.4 1472.3  1567.5 1762.8;
	1239.6 2483.2  1511.8 2654.3;
	]);
sg3 = SegGroup([
	1793.7 1633.7  2107.2 1775.8;
	1828.7 2705.6  2082.1 2752.6;
	1829.8 2330.3  2109.2 2406.7;
	1830.8 2305.0  2126.7 2388.5;
	1811.9 1426.4  2067.1 1554.6;
	]);
sg5 = SegGroup([
	2822.2 1539.9  2690.8 1721.2;
	2909.9 2748.7  3033.9 2688.8;
	2902.8 2670.4  3032.5 2598.6;
	2879.0 1830.3  2782.7 1936.1;	
	2822 2220  2934 2118;
	2879 1831  2784 1934;
	]);

% adds the given segments of the plane PI
sg2 = sg2 + s2;
sg3 = sg3 + s3;
sg5 = sg5 + s5;

sg2.draw;
sg3.draw;
sg5.draw;
%%
limit = Seg(HX(1.3*[0,size(img,1)]),HX(1.3*size(img,2,1))).line;

sg2.draw_to(limit);
sg3.draw_to(limit);
sg5.draw_to(limit);

%% Compute the vanish points
% searches the best approximation for the vanish points 
% of the facades' planes with LSM
v2 = sg2.find_vanish_point;
v3 = sg3.find_vanish_point;
v5 = sg5.find_vanish_point;

v2.draw_point;
v3.draw_point;
v5.draw_point;

%% Compute the line at the infinity
% prepares the matrix v_inf for LST
% to find the best approximation of infinity line l_inf
v_inf = [v3.X v5.X v2.X]';

% computes the normalized data and the corresponding similar transformation
[T, v_inf_n] = get_normalized_transformation(v_inf);

% looks for the solution for l_inf that minimizing ||v_inf * l_inf||
[~, ~, V] = svd(v_inf_n);

v = V(:, end);
v = v / v(end);

% reverts the data normalization
l_inf = T' * HX(v(1:end-1)');

l_inf.draw_line;

%% Compute the affine rectification matrix to send l_inf to its canonical position
H_p = [1 0 0; 0 1 0; l_inf.X.'];

%% Show the rectificated image
img_a = imwarp(img, projective2d(H_p.'));
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

% defines the shape of a single row of the 
a = @(l,m) [l(1)*m(1), l(1)*m(2)+l(2)*m(1), l(2)*m(2)];

C_d = [
	a(l1,l2);
	a(l4,l5);
	a(l5,l6);
	];

%% Searching the infinite line conic in affine rectificated image
% looks for the solution for s that minimizing ||C_d * s|| to find C_inf
[~, ~, V] = svd(C_d);

s = V(:,end);

C_inf = [[s(1) s(2) 0]; [s(2) s(3) 0]; [0 0 0]];
S = [[s(1) s(2)]; [s(2) s(3)]];

%% Looking for affine trasformation to map c_inf to its canonical position
[U,D,V] = svd(S);

H_a = diag([0 0 1]);
H_a(1:2,1:2) = U*sqrt(D)*V';

% invert H_a
H_a = eye(3) / H_a;

% removes the possible mirroring effect given by affine transformation
if H_a(1,1) < 0
	H_a = H_a * diag([-1 1 1]);
end
if H_a(2,2) < 0
	H_a = H_a * diag([1 -1 1]);
end

% computes the complessive homography
H = H_a * H_p;

%% Show the rectificated image
%Hi_a = rescale_h(H_a);
sameAsInput = affineOutputView(size(img_a),affine2d(H_a.'),'BoundsStyle','SameAsInput');

%Hi = rescale_h(H_a * H_p);
img_ap = imwarp(img, projective2d(H.'),'OutputView',sameAsInput);

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

H_t = [eye(2), -o.X(1:2);zeros(2,1)' 1];

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
% puts the segment s2 on the y-axis
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

figure, hold on, daspect([1 1 1]);
s1_s.draw;
s2_s.draw;
s3_s.draw;
s4_s.draw;
s5_s.draw;
s6_s.draw;

%% ######## G2 Calibration ########
figure, imshow(img), hold on;

%% Add veritical lines
sgv = SegGroup([
		1197 2226  1342 1191;
		3150 2953  2838 1346;
		1255 1181  975 2945;
		1531 2439  1574 1808;
		374 2003  512 1531;
		2769 2044  2839 2491;
	]);

sgv.draw;

%% Draw the extension of the vertical lines
limit = Seg(HX(-1.5*[0,size(img,1)]),HX(-1.7*size(img,2,1))).line;

sgv.draw_to(limit);

%% Compute the vertical vanish point
% exploits several vertical lines with LSM to reduce the error
vv = sgv.find_vanish_point;

vv.draw_point;

%% Draw vanish points for plane 2,3 and 5

v2.draw_point;
v3.draw_point;
v5.draw_point;

%%
%{
syms l1 l2 l3 x1 x2 x3;
syms w1 w2 w3 w4 w5;
plucker = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
W = [
	[w1 , 0, w3];
	[0, w2, w4];
	[w3, w4, w5];
	];

l = [l1 l2 l3].';
l_p = plucker(l);

A = l_p * W * [x1 x2 x3].';

collect(A, [w1 w2 w3 w4 w5])

A_p = @(l,v) [
	0, -l(3)*v(2), l(2)*v(1), l(2)*v(2)-l(3)*v(3), l(2)*v(3);
	l(3)*v(1), 0, l(3)*v(3)-l(1)*v(1), -l(1)*v(2), -l(1)*v(3);
	-l(2)*v(1), l(1)*v(2), -l(2)*v(3), l(1)*v(3), 0;
	];

A_a = collect(A_p(l, [x1 x2 x3].')*[w1 w2 w3 w4 w5].', [w1 w2 w3 w4 w5])
%}

%%
% consider the matrix W = (KK')^-1 and its elements w = [w1,w2,w3,w4,w5,w6]'
% w2 = 0 due to skew factor approsimation s = 0
% a'Wb = 0 constraints can be collected in Aw = 0
% where each of them can be write as a row of A
% a_i = [v1u1, v1u2+v2u1, v2u2, v1u3+v3u1, v2u3+v3u2, v3u3]

% a = @(v,u) [v(1)*u(1), v(1)*u(2)+v(2)*u(1), v(2)*u(2), v(1)*u(3)+v(3)*u(1), v(2)*u(3)+v(3)*u(2), v(3)*u(3)];

% imposes an hard contraint w2=0
% C = [
% 	[0 1 0 0 0 0];
% 	zeros([5,6]);
% 	];
% 
% [~, ~, V] = svd(C);
% C_p = V(:, 1+rank(C):end);

% gets a normalizes transformation to rescale point
% with the aim to reduce geometrical error in the extimation
T = get_normalized_transformation([vv.X, v2.X, v3.X, v5.X]');
%T = eye(3);

l_inf_r = inv(T)' * l_inf;

vv_r = T * vv;
% v2_r = T * v2;
% v3_r = T * v3;
% v5_r = T * v5;

% uses only the homography for the metric rectification
h = H_a * H_p;
h = T * h / T;
h = inv(h);
h = h/h(3,3);

% structure for the constraint v W u = 0 in the form a' w = 0
a = @(v,u) [v(1)*u(1), v(2)*u(2), v(1)*u(3)+v(3)*u(1), v(2)*u(3)+v(3)*u(2), v(3)*u(3)];

% structure for the constraint [l]_x W v = 0 in the form A w = 0 
A_p = @(l,v) [
	0, -l(3)*v(2), l(2)*v(1), l(2)*v(2)-l(3)*v(3), l(2)*v(3);
	l(3)*v(1), 0, l(3)*v(3)-l(1)*v(1), -l(1)*v(2), -l(1)*v(3);
	-l(2)*v(1), l(1)*v(2), -l(2)*v(3), l(1)*v(3), 0;
	];

A = [
	a(h(:,1), h(:,2));
 	a(h(:,1), h(:,1)) - a(h(:,2), h(:,2));
	A_p(l_inf_r.X, vv_r.X);
% 	a(vv_r.X, v3_r.X);
% 	a(vv_r.X, v5_r.X);
% 	a(vv_r.X, v2_r.X);
	];

%%%
% [~, ~, V] = svd(A*C_p);
% 
% w = C_p*V(:,end);

[~, ~, V] = svd(A);
w = V(:,end);

W = [
	[w(1) , 0, w(3)];
	[0, w(2), w(4)];
	[w(3), w(4), w(5)];
	];

if all(eigs(W) < 0)
	W = -W;
end

eigs(W)

K = inv(chol(W));
K = T \ K;
%K = K * T;

K = K / K(3,3);

%% ######## G3 Localization ########

H = inv(H_r * H_t * H_a * H_p);

B = K \ H;
B = B / norm(B)

Rt = [B(:,1), B(:,2), cross(B(:,1), B(:,2)), B(:,3)]

M = K*Rt;

% finds camera center in the world reference frame
c = null(M);
c = c/c(4);
c = HX(c(1:3));

HX.sdraw_point(M * c);

%%
figure, imshow(img_g), hold on;

x_axis = M * Seg(HX([0 0 0]), HX([500 0 0]));
y_axis = M * Seg(HX([0 0 0]), HX([0 500 0]));
z_axis = M * Seg(HX([0 0 0]), HX([0 0 5000000]));

x_axis.draw("Color", "red");
y_axis.draw("Color", "green");
z_axis.draw("Color", "blue");

%%
s1.draw;
s2.draw;
s3.draw;
s4.draw;
s5.draw;
s6.draw;
HX.sdraw_point(M * HX([0 0 0]));
HX.sdraw_point(M * HX([0 0 0]));
HX.sdraw_point(M * HX([0 1623 0]));
HX.sdraw_point(M * HX([2951 2851 0]));
HX.sdraw_point(M * HX([0 0 -10000000]));
%HX.sdraw_point(M * HX([0 0 0]), "Color", "c");
%HX.sdraw_point(M * HX([0 0 0]), "Color", "y");
%HX.sdraw_point(M * HX([0 0 0]), "Color", "k");


%%