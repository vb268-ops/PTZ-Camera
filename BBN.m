%Vaibhav Bisht
%MS, Mechanical Engineering, Cornell University

%Program: BBN FOR TARGET SEARCH

clc
clear all

%Coordinates of pinhole in Inertial Frame
xi = 2.54; yi = 2.2; zi = 3.18;

%Matrix with pinhole coordinates
Oa = [xi;yi;zi];

%Camera dimensions
a=0.036; b=0.027; lambda=0.05;

%Image of PTZ camera for used for video of tracking operation
I = imread('ptz.jpg');

%Creating set of target positions from instance (k-1) to (k+2)
for i=1:4
    if i==4
        x(i) = x(i-1) + 2;
        y(i) = x(i-1) + 2;
        z(i) = 0;
    elseif i==1
        x(i) = xi;
        y(i) = yi;
        z(i) = 0;
    else
        x(i) = x(i-1) + 0.5;
        y(i) = y(i-1) + 0.5;
        z(i) = 0;
    end
end

%Conditional probabilities or different cases of Δx and Δy
for i=1:4
    if i==1
        p_q_xp_yp(i) = 0.94;
    else
        p_q_xp_yp(i) = 0.2;
    end
    if i==2
        p_q_xn_yp(i) = 0.94;
    else
        p_q_xn_yp(i) = 0.02;
    end
    if i==3
        p_q_xn_yn(i) = 0.94;
    else
        p_q_xn_yn(i) = 0.02;
    end
    if i==4
        p_q_xp_yn(i) = 0.94;
    else
        p_q_np_yn(i) = 0.02;
    end
end
        
%Computing Δx and Δy for test case
delta_x = x(3) - x(2);
delta_y = y(3) - y(2);

if (delta_x>0 || delta_x<0) && (delta_y>0 || delta_y<0)
%Finding Qaudrant using conditional probability once Δx and Δy realised
    if delta_x>0 && delta_y>0
        quadrant = find(p_q_xp_yp == 0.94);
    elseif delta_x>0 && delta_y<0
        quadrant = find(p_q_xp_yn == 0.94);
    elseif delta_x<0 && delta_y<0
        quadrant = find(p_q_xn_yn == 0.94);
    elseif delta_x<0 && delta_y>0
        quadrant = find(p_q_xn_yp == 0.94);
    end
end

%Deciding angle based on quadrant
for i=1:4
    if i<4
        psi(i) = (pi*90)/180;
        phi(i) = (pi*180)/180;
    elseif i==4
        if quadrant==1
            psi(i) = (pi/180)*135;
            phi(i) = (pi/180)*135;
        elseif quadrant==2
            psi(i) = (pi/180)*45;
            phi(i) = (pi/180)*225;
        elseif quadrant==3
            psi(i) = (pi/180)*135;
            phi(i) = (pi/180)*225;
        elseif quadrant==4
            psi(i) = (pi/180)*45;
            phi(i) = (pi/180)*135;
        end
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

    %Coordinates of image sensors in inertial frame
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
    
    %Deleting matrices for next iteration
    hpsi(1,:) = [];
    hpsi(2,:) = [];
    hphi(1,:) = [];
    hphi(2,:) = [];
        
    %Preparing for Patch in animation
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

for i=1:4
    %Plotting Origin
    h1 = plot3(xi,yi,zi,'k*');
    xlabel('X-Axis (Tilt)');
    ylabel('Y-Axis');
    zlabel('Z-Axis (Pan)');
    title('Target Search')
    xlim([-5.46 10.54]);
    ylim([-5.8 10.2]);
    zlim([0 5]);
    hold on
    
    %Plotting Target
    h2 = plot3(x(i), y(i), z(i), 'k.', 'MarkerSize', 20);
    
    %Plotting the FOV (z1, z2, z3, z4)
    h3 = plot3([z1(1,i) z2(1,i)], [z1(2,i) z2(2,i)], [0 0], 'b');
    h4 = plot3([z2(1,i) z3(1,i)], [z2(2,i) z3(2,i)], [0 0], 'b');
    h5 = plot3([z3(1,i) z4(1,i)], [z3(2,i) z4(2,i)], [0 0], 'b');
    h6 = plot3([z4(1,i) z1(1,i)], [z4(2,i) z1(2,i)], [0 0], 'b');
    
    %Patch
    h7 = patch(xf1(:,i), yf1(:,i), zf1(:,i), 'blue', 'FaceAlpha', 0.2);
    h8 = patch(xf2(:,i), yf2(:,i), zf2(:,i), 'blue', 'FaceAlpha', 0.2);
    h9 = patch(xf3(:,i), yf3(:,i), zf3(:,i), 'blue', 'FaceAlpha', 0.2);
    h10 = patch(xf4(:,i), yf4(:,i), zf4(:,i), 'blue', 'FaceAlpha', 0.2);
    
    %Time Step
    if i==1
        str1 = {'k-1'};
        h11 = text(6, 8, 0, str1);
    elseif i==2
        str2 = {'k'};
        h11 = text(6, 8, 0, str2);
    elseif i==3
        str3 = {'k+1'};
        h11 = text(6, 8, 0, str3);
    elseif i==4
        str4 = {'k+2'};
        h11 = text(6, 8, 0, str4);
    end
    
    %Target Messages
    if i<length(x)-1
        h12 = patch(xfb(:,i), yfb(:,i), zfb(:,i), 'yellow', 'FaceAlpha', 0.5);
        str5 = {'Target Inside FOV'};
        h13 = text(-2, 1, 5, str5);
        str6 = {'All Well!'};
        h14 = text(2, -4, 0, str6);
    elseif i==(length(x)-1)
        h12 = patch(xfb(:,i), yfb(:,i), zfb(:,i), 'red', 'FaceAlpha', 0.5);
        str7 = {'Target Lost!'};
        h13 = text(-2, 1, 5, str7, 'Color', 'r');
        str8 = {'Using BBN for Target Search'};
        h14 = text(2, -4, 0, str8);
     elseif i>(length(x)-1)
        h12 = patch(xfb(:,i), yfb(:,i), zfb(:,i), 'yellow', 'FaceAlpha', 0.5);
        str9 = {'Target Found in Quadrant' quadrant};
        h13 = text(-2, 1, 5, str9);
        str10 = {'All Well Again'};
        h14 = text(4, -4, 0, str10);
    end
    
    %Coordinates of image to be added on 3D plot
    a = [2.54 2.54; 2.54 2.54];
    b = [3 1; 3 1];
    c = [4.78 4.78; 2.78 2.78];
    
    %Inserting image of PTZ camera at pinhole
    h15 = surf(a,b,c,'CData',I,'FaceColor','texturemap');
    
    pause(1);
    
    movieVector(i) = getframe;
    delete([h1 h2 h3 h4 h5 h6 h7 h8 h9 h10 h11 h12 h13 h14 h15]);
end

%Recording video 
myWriter = VideoWriter('ZLS_Movie','MPEG-4'); 
warning('off') 
myWriter.FrameRate = 1; 
open(myWriter); 
writeVideo(myWriter,movieVector); 
close(myWriter);