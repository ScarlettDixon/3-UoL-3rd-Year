% MATLAB script for Assessment Item-1
% Task-2
% Functions not allowed: fspecial, imfilter, conv2, medfilt2 and filter2
% Notes:
% Use Zero Padding to deal with edges
clear; close all; clc;

% Step-1: Load input image
J = imread('Noisy.png');
subplot(2,2,1);
imshow(J);
title('Step-1: Load input image');

% Step-2: Conversion of input image to grey-scale image
Jgray = rgb2gray(J);
subplot(2,2,2);
imshow(Jgray);
title('Step-2: Conversion of input image to greyscale');

% Step-3: Smooth using averaging with 5x5 kernel
JSmoAvg = Jgray;
JSmoAvg2 = Jgray;
JSmoAvg = padarray(JSmoAvg,[2 2]); %Padd with Two lines of zeros

% mask = ones(5);
% mask_nxn = 5;
% i = size(JSmoAvg,1)-5;
% j = size(JSmoAvg,2)-5;
% sum = 0;
%  for row = 1:mask_nxn
%  	for col = 1:mask_nxn
%  		sum = sum +(1/25)*JSmoAvg(row:(row+i), col:(col+j)); 
%  	end
%  end
% sum = round(sum,0);
% imshow(sum);
i = size(JSmoAvg,1)-2; %Two less to stay within the bounds of the original image
j = size(JSmoAvg,2)-2;
for X = 3:i
    for Y = 3:j %Start at three to stay within the bounds of the original image
        sum = double(0);
        for countX = -2:2 
            for countY = -2:2 %Go across the 5x5 grid in a for loop
                sum = sum + double(JSmoAvg((X + countX), (Y + countY))); %Add up all variables within the mask
            end
        end
        Avg = (1/25)* sum; %Mean is the sum of all
        JSmoAvg2(X-2, Y-2) = Avg; 
    end
end
subplot(2,2,3);
imshow(JSmoAvg2);
title('Step-3: Smooth using averaging with 5x5 kernel');

% Step-4: Smooth using median with 5x5 kernel
JSmoMed = Jgray;
JSmoMed2 = Jgray;
JSmoMed = padarray(JSmoMed,[2 2]); %Padd with Two lines of zeros
i = size(JSmoMed,1)-2;
j = size(JSmoMed,2)-2;
Table = uint8(zeros(1,25));
count = 1;
for X = 3:i
    for Y = 3:j
        for countX = -2:2
            for countY = -2:2
                Table(1, count) = JSmoMed((X + countX), (Y + countY)); %Add all variables from mask into table
                count = count +1;
            end
        end
        Table = sort(Table); %Sort the table for easier access to the median
        count = 1; %Reset count
        JSmoMed2(X-2,Y-2) = Table(1,13); % 13 is midway through 25 and so the median value of the sorted list
    end
end
subplot(2,2,4);
imshow(JSmoMed2);
title('Step-4: Smooth using median with 5x5 kernel');


% Checking they are correct
% h = fspecial('average', 5);
% FilteredImage = imfilter(Jgray,h);
% subplot(2,2,3);
% imshow(FilteredImage);
% 
% FilteredImage2 = medfilt2 (Jgray,[5 5], 'zeros');
% subplot(2,2,4);
% imshow(FilteredImage2);

%References:
%Workshop tasks