clear
clc
close all;

%% Setting up the Udp Connection
% ipA = 'localhost';    portA = 9090;
% ipB = 'localhost';    portB = 9091;
% udpB = udp(ipA,portA,'LocalPort',portB);
% fopen(udpB);

%% Loading the intrinsic camera parameters
load('camparam.mat');
intpar=(cameraParams.IntrinsicMatrix)';
focal=cameraParams.FocalLength(1);
Fu=intpar(1,1);
Fv=intpar(2,2);
u0=intpar(1,3);
v0=intpar(2,3);

Yc=210; % Mounting Height
Cpos=60;% Camera Position

%% Read the video
mov=VideoReader('Clockwise.mp4','CurrentTime',0);

%% Uncomment when writing to video
% write=VideoWriter('Final HMI');
% write.FrameRate = 10.15;
% open(write);

%% Main Loop
while hasFrame(mov)
    pic=readFrame(mov);
    pic=imresize(pic,0.25);
    shape = size(pic);
    
    % Setting RGB limits for the tape
    rlim1=30;glim1=85;blim1=100; % Low Limits
    rlim2=106;glim2=160;blim2=255; % High Limits
    
    % Color Selection for lane detection
    [x,y] = find(pic(:,:,1)<rlim1|pic(:,:,2)<glim1|pic(:,:,3)<blim1|...
        pic(:,:,1)>rlim2|pic(:,:,2)>glim2|pic(:,:,3)>blim2);
    [x2,y2] = find(rlim2>pic(:,:,1)>rlim1 | glim2>pic(:,:,2)>glim1...
        | blim2>pic(:,:,3)>blim1);
    nl_pic = pic;
    for i =1:length(x)
        nl_pic(x(i), y(i),:)=[0, 0, 0];    
        % make all colors below & above lim as 0
    end
    
    for i =1:length(x2)
        nl_pic(x2(i), y2(i),:)=[255, 255, 255];    
        % make all colors between lim as 255
    end
    
    % Edge Detection
    gray_pic=rgb2gray(pic);
    [~,thr1] = edge(gray_pic,'canny');
    thr = thr1*0.8;
    edge_pic = edge(gray_pic,'canny',thr);

    % Region Masking
    a=[shape(2)*0.3,shape(2)*0.7,shape(2)*0.9, 0.1*shape(2)];
    y_mul=0.55;
    b=[shape(1)*y_mul,shape(1)*y_mul,shape(1),shape(1)];
    bw=roipoly(pic,a,b);    % Binary image masked by polygon
    
    % Combination of Edge Detection, Color Selection and Region Masking 
    BP=(nl_pic(:,:,1)&bw | edge_pic(:,:,1)&bw | nl_pic (:,:,2)&bw...
        | nl_pic (:,:,3)&bw);
    
    % Hough Transform and Hough Lines to find prominent lines for lanes in
    % the above image
    [H,T,R] = hough(BP);
    P = houghpeaks(H,100);
    lines = houghlines(BP,T,R,P,'FillGap',4,'MinLength',30);
    
    % Uncomment to see Binary edge detected image
%     figure(1);
%     imshow(BP)
%     hold on;
%     for i= 1:length(lines)
%         plot([lines(i).point1(1),lines(i).point2(1)],[lines(i).point1(2)...
%             ,lines(i).point2(2)],'LineWidth',2,'Color','red');
%     end
%     hold off;
    
    % Identifying Left and Right Lines for lane boundaries using angular
    % threshold and 'polyfit' function
    anglethres=tand(50); %separate left/right by orientation threshold
    leftlines=[];rightlines=[]; %Two group of lines
    for k = 1:length(lines)
        x1=lines(k).point1(1);y1=lines(k).point1(2);
        x2=lines(k).point2(1);y2=lines(k).point2(2);
        if (x2>=shape(2)/2) && ((y2-y1)/(x2-x1)>anglethres)
            rightlines=[rightlines;x1,y1;x2,y2];
        elseif (x2<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*anglethres))
            leftlines=[leftlines;x1,y1;x2,y2];
        end
    end
    draw_y=[shape(1)*y_mul,shape(1)]; %two row coordinates
    
    % Allows the code not to stop if it is unable to find either of the
    % lines
    if not(isempty(leftlines))
        PL=polyfit(leftlines(:,2),leftlines(:,1),1);
        draw_lx=polyval(PL,draw_y); %two col coordinates of left line
    else
        draw_lx=[shape(2)/3;shape(2)/3];
    end
    if not(isempty(rightlines))
        PR=polyfit(rightlines(:,2),rightlines(:,1),1);
        draw_rx=polyval(PR,draw_y); %two col coordinates of right line
    else
        draw_rx=[shape(2)*2/3;shape(2)*2/3];
    end
    
    % Final Figure Part 1: Image with lane detected
    figure(2);
    imagesc(pic);
    hold on;
    plot(draw_lx,draw_y,'LineWidth',2,'Color','red');
    hold on;
    plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
    hold on;
    
    % Extracted Lane Marker Points
    Zcl=(Fv*Yc)./(draw_y-v0);
    Xcl=Zcl.*(draw_lx-u0)/Fu;
    Xlf=Xcl(1);
    Xln=Xcl(2);
    Zlf=Zcl(1);
    Zln=Zcl(2);
    
    Zcr=(Fv*Yc)./(draw_lx-v0);
    Xcr=Zcr.*(draw_y-u0)/Fu;
    Xrf=Xcr(1);
    Xrn=Xcr(2);
    Zrf=Zcr(1);
    Zrn=Zcr(2);
    
    % CenterLine
    Xcf=0.5*(Xrf+Xlf);
    Zcf=0.5*(Zrf+Zlf);
    Xcn=0.5*(Xrn+Xln);
    Zcn=0.5*(Zrn+Zln);
    
    % Stanley Controller
    Te=-atand((Zcf-Zcn)/(Xcf-Xcn))-90;          % Departure Angle
    efa=Xcn+((Xcf-Xcn)/(Zcf-Zcn))*(Zcn-Cpos);   % Departure Distance
    
    k1=1; k2=0.038;     % Controller Gain

    phi=-k1*Te-k2*efa;    % Steering Angle
    
    % Final Figure Part 2: HMI for steering
    if phi>2.3
        plot(shape(2)/5,shape(1)/3,'<','LineWidth',16,'Color','red');
        plot(shape(2)/5+28,shape(1)/3,'s','LineWidth',16,'Color','red');
    elseif phi<-0.7
        plot(shape(2)*4/5,shape(1)/3,'>','LineWidth',16,'Color','red');
        plot(shape(2)*4/5-28,shape(1)/3,'s','LineWidth',16,'Color','red');
    elseif phi<2.3 && phi>-0.7
        plot(shape(2)/2,shape(1)/3,'^','LineWidth',16,'Color','red');
        plot(shape(2)/2,shape(1)/3+16,'s','LineWidth',16,'Color','red');
    end
    hold on;
    
    % Final Figure Part 3: Centerline for camera(vehicle) frame
    plot([shape(2)/2,shape(2)/2],[shape(1),0],'--','LineWidth',2,'Color','white');
    hold on;
    
    % Final Figure Part 4: Centerline for the lane
    plot(((draw_lx+draw_rx)/2),draw_y,'LineWidth',2,'Color','black');
    hold on;
    
    % Udp Communication received from Road Sign Recognition
%     msg = fscanf(udpB);
    
    % Final Figure Part 5: Display Controls using different color blocks
    % and text.
%     if contains(msg,"sc")
%         plot(shape(2)*4/5,shape(1)*2/3,'^','LineWidth',40,'Color','yellow');
%         text(shape(2)*4/5,shape(1)*2/3+40,'SLOW DOWN','HorizontalAlignment','center','Color','yellow','FontSize',16);
%     elseif contains(msg,"st")
%         plot(shape(2)*4/5,shape(1)*2/3,'s','LineWidth',40,'Color','red');
%         text(shape(2)*4/5,shape(1)*2/3+40,'STOP','HorizontalAlignment','center','Color','red','FontSize',16);
%     else
%         plot(shape(2)*4/5,shape(1)*2/3,'^','LineWidth',40,'Color','green');
%     end
%     
%     hold off;
    
    % Uncomment while writing video
%     F=getframe;
%     fig=frame2im(F);
%     writeVideo(write,fig);
    pause(0.0001);
end

% Uncomment while writing video
% close(write);

%% Close Udp Communication
% fclose(instrfindall);