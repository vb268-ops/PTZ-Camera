%Vaibhav Bisht
%MS, Mechanical Engineering, Cornell University

%Program: DETECTION 

clc
clear all

%Origin 
xi=2.54; yi=2.2; zi=3.18;
Oa=[xi;yi;zi];

%Camera Dimensions
a=0.036; b=0.027; lambda=0.05;

%Image of PTZ camera for used for video of tracking operation
I = imread('ptz.jpg');

%Number of Iterations
l=14;

%Movement of Camera
for i=1:l
    if i==1
        xt(i) = 0.34;
        yt(i) = 2.2;
    else
        xt(i) = xt(i-1) + 0.3;
        yt(i) = 2.2 + (((-1)^i)*(i/10));
    end
end

%Movement of Target
for i=1:l
    if i==1
        xd(i) = 2.54;
        yd(i) = -0.6;
    else
        xd(i) = xd(i-1) + ((-1)^i * i/5);
        yd(i) = yd(i-1) + 0.5;
    end
end

for i=1:l
    %Pan angle in different iterations
    ppsi(i) = yt(i)-yi;
    bpsi(i) = xt(i)-xi;
    psi(i) = ((90*pi)/180) + atan(ppsi(i)/bpsi(i));     
    
    %Tilt angle in different iterations
    if xt(i)<xi
        bphi(i) = zi;
        pphi(i) = sqrt((xt(i)-xi)^2 + (yt(i)-yi)^2);
        phi(i) = (((180*pi)/180) + atan(pphi(i)/bphi(i)));
    else
        bphi(i) = zi;
        pphi(i) = sqrt((xt(i)-xi)^2 + (yt(i)-yi)^2);
        phi(i) = (((180*pi)/180) - atan(pphi(i)/bphi(i)));
    end
    
    %Rotation Matrices
    hpsi = [cos(psi(i)), -sin(psi(i)), 0; sin(psi(i)), cos(psi(i)), 0; 0, 0, 1];
    hphi = [1, 0, 0; 0, cos(phi(i)), -sin(phi(i)); 0, sin(phi(i)), cos(phi(i))];
    
    q1v(:,i) = [a/2; b/2];
    q2v(:,i) = [a/2; -b/2];
    q3v(:,i) = [-a/2; -b/2];
    q4v(:,i) = [-a/2; b/2];
    
    %Image sensor coordinates in body frame
    q1b(:,i) = [q1v(:,i); lambda];
    q2b(:,i) = [q2v(:,i); lambda];
    q3b(:,i) = [q3v(:,i); lambda];
    q4b(:,i) = [q4v(:,i); lambda];

    %Coordinates of image sensor in inertial frame
    q1i(:,i) = hpsi*hphi*q1b(:,i);
    q2i(:,i) = hpsi*hphi*q2b(:,i);
    q3i(:,i) = hpsi*hphi*q3b(:,i);
    q4i(:,i) = hpsi*hphi*q4b(:,i);
    
    %Dilating factor
    pho_1_4(i) = -zi/(((b/2)*sin(phi(i))) + (lambda*cos(phi(i))));
    pho_2_3(i) = -zi/(((-b/2)*sin(phi(i))) + (lambda*cos(phi(i))));

    %Implemeting dilating factor
    q1di(:,i) = pho_1_4(i)*q1i(:,i);
    q2di(:,i) = pho_2_3(i)*q2i(:,i);
    q3di(:,i) = pho_2_3(i)*q3i(:,i);
    q4di(:,i) = pho_1_4(i)*q4i(:,i);
    
    %Coordinates of FOV in inertial frame
    z1(:,i) = Oa + q1di(:,i);
    z2(:,i) = Oa + q2di(:,i);
    z3(:,i) = Oa + q3di(:,i);
    z4(:,i) = Oa + q4di(:,i);
    
    %Check if Target is inside FOV
    alpha_x = [z1(1,i); z2(1,i); z3(1,i); z4(1,i)]; 
    alpha_y = [z1(2,i); z2(2,i); z3(2,i); z4(2,i)]; 
    shp = alphaShape(alpha_x, alpha_y);
    test(i) = inShape(shp, xd(i), yd(i));
    
    %Deleting matrices for next iteration
    hpsi(1,:) = [];
    hpsi(2,:) = [];
    hphi(1,:) = [];
    hphi(2,:) = [];
    alpha_x = [];
    alpha_y = [];
    
    %Preparing for Patch
    xf1(:,i) = [z1(1,i) z2(1,i) xi];
    yf1(:,i) = [z1(2,i) z2(2,i) yi];
    zf1(:,i) = [z1(3,i) z2(3,i) zi];

    xf2(:,i) = [z2(1,i) z3(1,i) xi];
    yf2(:,i) = [z2(2,i) z3(2,i) yi];
    zf2(:,i) = [z2(3,i) z3(3,i) zi];

    xf3(:,i) = [z3(1,i) z4(1,i) xi];
    yf3(:,i) = [z3(2,i) z4(2,i) yi];
    zf3(:,i) = [z3(3,i) z4(3,i) zi];

    xf4(:,i) = [z4(1,i) z1(1,i) xi];
    yf4(:,i) = [z4(2,i) z1(2,i) yi];
    zf4(:,i) = [z4(3,i) z1(3,i) zi];
    
    xfb(:,i) = [z1(1,i) z2(1,i) z3(1,i) z4(1,i)];
    yfb(:,i) = [z1(2,i) z2(2,i) z3(2,i) z4(2,i)];
    zfb(:,i) = [z1(3,i) z2(3,i) z3(3,i) z4(3,i)];
end

%Animation
for i=1:l
    %Image Point
    h1 = plot3(xd(i), yd(i), 0, 'k.', 'MarkerSize', 20);
    title('Detection')
    xlim([-5.46 10.54]);
    ylim([-5.8 10.2]);
    zlim([0 5]);
    hold on
    
    %Lines from Origin to Projection Plane
    h2 = plot3([xi z1(1, i)], [yi z1(2,i)], [zi z1(3,i)], 'r'); 
    xlabel('X-Axis (Tilt)');
    ylabel('Y-Axis');
    zlabel('Z-Axis (Pan)');
    h3 = plot3([xi z2(1, i)], [yi z2(2,i)], [zi z2(3,i)], 'r');
    h4 = plot3([xi z3(1, i)], [yi z3(2,i)], [zi z3(3,i)], 'r');
    h5 = plot3([xi z4(1, i)], [yi z4(2,i)], [zi z4(3,i)], 'r');
    
    %Lines of Projection Plane
    h6 = plot3([z1(1,i) z2(1,i)], [z1(2,i) z2(2,i)], [0 0], 'b');
    h7 = plot3([z2(1,i) z3(1,i)], [z2(2,i) z3(2,i)], [0 0], 'b');
    h8 = plot3([z3(1,i) z4(1,i)], [z3(2,i) z4(2,i)], [0 0], 'b');
    h9 = plot3([z4(1,i) z1(1,i)], [z4(2,i) z1(2,i)], [0 0], 'b');
    
    %Patch
    h10 = patch(xf1(:,i), yf1(:,i), zf1(:,i), 'yellow', 'FaceAlpha', 0.2);
    h11 = patch(xf2(:,i), yf2(:,i), zf2(:,i), 'yellow', 'FaceAlpha', 0.2);
    h12 = patch(xf3(:,i), yf3(:,i), zf3(:,i), 'yellow', 'FaceAlpha', 0.2);
    h13 = patch(xf4(:,i), yf4(:,i), zf4(:,i), 'yellow', 'FaceAlpha', 0.2);
    h14 = patch(xfb(:,i), yfb(:,i), zfb(:,i), 'red', 'FaceAlpha', 0.8);
    
    %Printing pan and tilt
    ps(i) = (180/pi)*psi(i);
    ph(i) = (180/pi)*phi(i);
    str1 = {'Pan: Ψ (°)' ps(i)};
    str2 = {'Tilt: Φ (°)' ph(i)};
    h15 = text(0, 10, 0, str1);
    h16 = text(0, 10, 1, str2);
    
    if test(i) == 1
        str3 = {'Target inside FOV'};
        h17 = text(5, 0.2, 0, str3, 'Color', 'g');
    elseif test(i) == 0
        str3 = {'Target outside FOV'};
        h17 = text(5, 0.2, 0, str3, 'Color', 'r');
    end
    
    %Coordinates of image to be added on 3D plot
    a = [1.5 3.5; 1.5 3.5];
    b = [3 1; 3 1];
    c = [4.68 4.68; 2.88 2.88];

    %Inserting image of PTZ camera at pinhole
    h18 = surf(a,b,c,'CData',I,'FaceColor','texturemap');
%     pause(0.5)
    movieVector(i) = getframe;
    
    delete([h1 h2 h3 h4 h5 h6 h7 h8 h9 h10 h11 h12 h13 h14 h15 h16 h17 h18]);
end

%Recording video 
myWriter = VideoWriter('Detection','MPEG-4'); 
warning('off') 
myWriter.FrameRate = 1; 
open(myWriter); 
writeVideo(myWriter,movieVector); 
close(myWriter);