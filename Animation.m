%Vaibhav Bisht
%MS, Mechanical Engineering, Cornell University

%Program: SIMPLE ANIMATION OF FOV AT PAN=105° & TILT=165°

clc
clear all

%Origin, Target & Camera Dimensions
xi=2.54; yi=2.2; zi=3.18;

%Matrix with pinhole coordinates
Oa = [xi;yi;zi];

%Camera dimensions
a=0.036; b=0.027; lambda=0.05;
x=[xi;yi;zi];

%Image of PTZ camera for used for animation
I = imread('ptz.jpg');

%Input angles
%Pan & Tilt
psi = (pi/180)*105;
phi = (pi/180)*165;

%Rotation Matrices
hpsi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
hphi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];

q1v = [a/2; b/2];
q2v = [a/2; -b/2];
q3v = [-a/2; -b/2];
q4v = [-a/2; b/2];

%Image sensor coordinates in body frame
q1b = [q1v; lambda];
q2b = [q2v; lambda];
q3b = [q3v; lambda];
q4b = [q4v; lambda];

%Coordinates of image sensors in inertial frame
q1i = hpsi*hphi*q1b;
q2i = hpsi*hphi*q2b;
q3i = hpsi*hphi*q3b;
q4i = hpsi*hphi*q4b;

%Dilating factor
pho_1_4 = -zi/(((b/2)*sin(phi)) + (lambda*cos(phi)));
pho_2_3 = -zi/(((-b/2)*sin(phi)) + (lambda*cos(phi)));

%Implemeting dilating factor
q1di = pho_1_4*q1i;
q2di = pho_2_3*q2i;
q3di = pho_2_3*q3i;
q4di = pho_1_4*q4i;

%Coordinates of FOV in inertial frame
z1 = Oa + q1di;
z2 = Oa + q2di;
z3 = Oa + q3di;
z4 = Oa + q4di;

%Preparing for Patch
xf1 = [z1(1) z2(1) xi];
yf1 = [z1(2) z2(2) yi];
zf1 = [z1(3) z2(3) zi];

xf2 = [z2(1) z3(1) xi];
yf2 = [z2(2) z3(2) yi];
zf2 = [z2(3) z3(3) zi];

xf3 = [z3(1) z4(1) xi];
yf3 = [z3(2) z4(2) yi];
zf3 = [z3(3) z4(3) zi];

xf4 = [z4(1) z1(1) xi];
yf4 = [z4(2) z1(2) yi];
zf4 = [z4(3) z1(3) zi];
    
xfb = [z1(1) z2(1) z3(1) z4(1)];
yfb = [z1(2) z2(2) z3(2) z4(2)];
zfb = [z1(3) z2(3) z3(3) z4(3)];

%Plotting pinhole
h1 = plot3(xi,yi,zi,'k*');
xlabel('X-Axis');
ylabel('Y-Axis');
zlabel('Z-Axis');
xlim([0 8]);
ylim([0 8]);
zlim([0 5]);
hold on
    
%Plotting the FOV in inertial frame
h2 = plot3([z1(1) z2(1)], [z1(2) z2(2)], [0 0], 'r');
h3 = plot3([z2(1) z3(1)], [z2(2) z3(2)], [0 0], 'r');
h4 = plot3([z3(1) z4(1)], [z3(2) z4(2)], [0 0], 'r');
h5 = plot3([z4(1) z1(1)], [z4(2) z1(2)], [0 0], 'r');
    
%Patch
h6 = patch(xf1, yf1, zf1, 'yellow', 'FaceAlpha', 0.2);
h7 = patch(xf2, yf2, zf2, 'yellow', 'FaceAlpha', 0.2);
h8 = patch(xf3, yf3, zf3, 'yellow', 'FaceAlpha', 0.2);
h9 = patch(xf4, yf4, zf4, 'yellow', 'FaceAlpha', 0.2);
h10 = patch(xfb, yfb, zfb, 'blue', 'FaceAlpha', 0.6);

%Displaying Angle
str1 = {'Pan: Ψ (°)' (psi*180)/pi};
str2 = {'Tilt: Φ (°)' (phi*180)/pi};
h11 = text(4, 6, 2, str1);
h12 = text(4, 6, 0, str2);

%Coordinates at which image 'I' will be placed
a = [2.54 2.54; 2.54 2.54];
b = [3 1; 3 1];
c = [4.78 4.78; 2.78 2.78];

%Plot of image in coordinate system
h13 = surf(a,b,c,'CData',I,'FaceColor','texturemap');

hold off