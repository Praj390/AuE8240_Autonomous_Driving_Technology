clear;
clc;
close all;

%% Start Udp communication
ipA = 'localhost';    portA = 9090;
ipB = 'localhost';    portB = 9091;
udpA = udp(ipB,portB,'LocalPort',portA);
fopen(udpA);

% Load Trained neural network
load net01.mat

% Read video as sequence of images
mov=VideoReader('Clockwise.mp4','CurrentTime',0);
t=0; % Frame Counter

while hasFrame(mov)
    pic=readFrame(mov);
    pic=imresize(pic,0.25);
    
    % Use the neural network to detect signs from the sequence of images
    [bboxes,score,label] = detect(rcnn,pic,'MiniBatchSize',128);
    % Gives the maximum score and index of the identified sign
    [score, idx] = max(score);
    % Bounding Box
    bbox = bboxes(idx, :);
    % Annotation for bounding box
    annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
    
    % Udp communication
    if isempty(bbox)
        fprintf(udpA, "no");    
        % Send useless message to keep in sync with other code
        t=t+1;
    else
        disp(annotation);
        disp(t);
        msg = label(idx);   % Returns categorical array
        msg2 = string(msg); % Convert to string for transmission of message
        fprintf(udpA, msg2);
        t=t+1;
    end
    
    % Annotated image with bounding box and sign text
    outputImage = insertObjectAnnotation(pic, 'rectangle', bbox, annotation);
    
    % Uncomment while using only this code. Commented for faster execution.
%     imshow(outputImage);
    B = any(bboxes);
    pause(0.01)
end
fclose(instrfindall);