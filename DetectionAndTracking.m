%Vaibhav Bisht
%MS, Mechanical Engineering, Cornell University

%Program: DETECTION & TRACKING USING CLOSED-LOOP CONTROL

clc
clear all

%Origin 
xi=2.54; yi=2.2; zi=3.18;
x=[xi;yi;zi];

%Camera Dimensions
a=0.036; b=0.027; lambda=0.05;
xtt=2.5;

%Image of PTZ camera for used for video of tracking operation
I = imread('ptz.jpg');

%Number of Iterations
t = 12;

%Two loops run to create target position
for i=1:t
    if i==1
        X(i)=1.25;
        Y(i)=2.2;
    else 
        X(i) = X(i-1)+0.25;
        Y(i)=2.2;
    end
end
for i=1:t 
    dd(i) = 2.50 - X(i);
    if dd(i)<0
        X(i) = xtt + i/3;
        xt(i) = X(i);
        yt(i) = Y(i) + (-1)^i*(i/4); 
    else
        yt(i) = Y(i);
        xt(i) = X(i);
    end
end

for i=1:t
    %Pan angle for tracking
    ppsi(i) = yt(i)-yi;
    bpsi(i) = xt(i)-xi;
    psi(i) = ((90*pi)/180) + atan(ppsi(i)/bpsi(i));     
    %Tilt angle for tracking
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
    z1(:,i) = x + q1di(:,i);
    z2(:,i) = x + q2di(:,i);
    z3(:,i) = x + q3di(:,i);
    z4(:,i) = x + q4di(:,i);
    
    %Check if Target is inside FOV
    centroid_x(i) = (z1(1,i) + z2(1,i) + z3(1,i) + z4(1,i))/4;
    centroid_y(i) = (z1(2,i) + z2(2,i) + z3(2,i) + z4(2,i))/4;
    if abs(centroid_x(i)-xtt) <0.1
        A = [z1(1,i) z2(1,i) z3(1,i) z4(1,i)];
        z_min = min(A);
    end
    
    %Deleting matrices for next iteration
    hpsi(1,:) = [];
    hpsi(2,:) = [];
    hphi(1,:) = [];
    hphi(2,:) = [];
    
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
for i=1:t
    %Image Point
    if dd(i)<=0
        h1 = plot3(xt(i), yt(i), 0, 'k.', 'MarkerSize', 20);
        title('Detection & Tracking')
        xlim([-5.96 15.54]);
        ylim([-6.3 15.2]);
        zlim([0 5]);
    else
        h1 = plot3(xi, yi, 0, 'k.', 'MarkerSize', 20);
        title('Detection & Tracking')
        xlim([-5.96 15.54]);
        ylim([-6.3 15.2]);
        zlim([0 5]); 
    end
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
    h15 = text(0, 11, 1, str1);
    h16 = text(4, 11, 1, str2);
    if xt(i)<z_min
        str3 = {'Target Being Detected'};
        h17 = text(-1, 8, 4.25, str3, 'Color', 'c');        
    elseif xt(i)>=z_min && xt(i)<xtt
        str3 = {'Target Detected & Being Centered'};
        h17 = text(-1, 8, 4.25, str3, 'Color', 'r');
    elseif abs(xt(i)-xtt)<0.1
        str3={'Target Centered in FOV & Ready to Track Movement'};
        h17 = text(-1, 8, 4.25, str3, 'Color', 'b');
    else
        str3 = {'Target Being Tracked'};
        h17 = text(-1, 8, 4.25, str3, 'Color', 'g');
    end
    
    %Coordinates of image to be added on 3D plot
%     a = [1 4; 1 4];
%     b = [3 1; 3 1];
%     c = [4.68 4.68; 2.88 2.88];
%     
%     %Inserting image of PTZ camera at pinhole
%     h18 = surf(a,b,c,'CData',I,'FaceColor','texturemap');
%     
    pause(1)
    movieVector(i) = getframe;
    
    delete([h1 h2 h3 h4 h5 h6 h7 h8 h9 h10 h11 h12 h13 h14 h15 h16 h17]);
end

%Recording video 
myWriter = VideoWriter('DetectionAndTracking','MPEG-4'); 
warning('off') 
myWriter.FrameRate = 1; 
open(myWriter); 
writeVideo(myWriter,movieVector); 
close(myWriter);