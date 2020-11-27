% MATLAB script for Assessment Item-1
% Task-1
% Functions not allowed: imresize and interp2
% Notes
% Image goes from 556x612 to 1668x1836, 3x Zoom
% Nearest Neighbour = Upping the size but keeping the same pixels
% Linear Interpolation = Getting the average of two known pixels (1D)
% Bilinear Interpolation = Getting the average of four known pixels (2D)
% Bilinear Interpolation = Involves doing 2 Linear Interpolations and then Linear Interpolating them
% 
clear; close all; clc;

% Step-1: Load input image
I = imread('Zebra.jpg'); % Size = 556x612
figure;
subplot(2,2,1);
imshow(I);
title('Step-1: Load input image');

% Step-2: Conversion of input image to grey-scale image
Igray = rgb2gray(I);
%figure;
subplot(2,2,2);
imshow(Igray);
title('Step-2: Conversion of input image to greyscale');

% Step-3: Enlarge using Nearest Neighbour
Zoom = 3; %To get the scale for the zoom image, we have to * by 1/3. This is the same as dividing by 3
INN = uint8(zeros(1668,1836));
%count = 0; count2 = 0; %Used for error checking
for rows = 1:1668 %Loop through all rows of larger image
	for cols = 1:1836 %Loop through all columns
		x = round((rows/Zoom), 0); %Default is zero anyway but better to be safe
		y = round((cols/Zoom), 0); %Dividing rows and columns by zoom number
		if (x < 1) % x or y may be rounded to zero if the original rows/cols value is one, these if statements resets them to one
			x = 1; 
		end
		if (y < 1)
			y = 1;
		end
		INN(rows, cols) = Igray(x,y); %New Image has pixels transferred from old Image
	end
end	
%figure;
subplot(2,2,3);
imshow(INN);
title('Step-3: Enlarge using Nearest Neighbour');

% Step-4: Enlarge using Bilinear interpolation
Zoom = 3; %place all original values into their new positions = 3n - 2 = Zoom*n - Distance between pixels
IBI = uint8(zeros(1668,1836));
for rows = 1:1668 %Loop through all rows
	for cols = 1:1836 %Loop through all columns
	x = rem ((rows/Zoom), 1); %Middle x-value, allowed to 1 decimal place
	x1 = floor((rows/Zoom)); %Lowest x-value
	x2 = ceil((rows/Zoom)); %Greatest x-value
	y = rem ((cols/Zoom), 1); %Middle y-value, allowed to 1 decimal place
	y2 = floor((cols/Zoom));%Lowest y-value
	y1 = ceil((cols/Zoom)); %Greatest y-value
	
	if (x1 < 1) % x1 or y1 may be rounded to zero if the original rows/cols value is one, these if statements resets them to 1
		x1 = 1; 
	end
	if (y2 < 1)
		y2 = 1;
	end
	
	Q11 = Igray(x1, y1);
	Q12 = Igray(x1, y2);
	Q21 = Igray(x2, y1);
	Q22 = Igray(x2, y2);
	
	%R1 = ((x2 - x)/(x2 - x1))*Q11 + ((x-x1)/ (x2 - x1))*Q21;
	%R2 = ((x2 - x)/(x2 - x1))*Q12 + ((x-x1)/ (x2 - x1))*Q22;
	R1 = Q21*x + Q11*(1-x); %Weight of Q21 and Q11 addedd together
    R2 = Q22*x + Q12*(1-x); %Weight of Q22 and Q12 addedd together
    
    %P11 = (((x2 - x)*(y2 - y))/((x2 - x1)*(y2 - y1)))*Q11;
    %P21 = (((x - x1)*(y2 - y))/((x2 - x1)*(y2 - y1)))*Q21;
    %P12 = (((x2 - x)*(y - y1))/((x2 - x1)*(y2 - y1)))*Q12;
    %P22 = (((x - x1)*(y - y1))/((x2 - x1)*(y2 - y1)))*Q22;
    %P = P11 + P21 + P12 + P22;
    P =R1*y + R2*(1-y); %Weight of R1 and R2 addedd together
	%P = ((y2 - y)/ (y2 - y1))*R1 + ((y - y1)/ (y2 - y1))*R2;
	IBI(rows, cols) = P; 
	end
end
%figure;
subplot(2,2,4);
imshow(IBI);
title('Step-4: Enlarge using Bilinear interpolation');

% Step-5: Zoomed in comparison
figure; %Used to display both subplots in one go
INNZoom = Zoomin(INN, 900,600,1200,900);
subplot(1,2,1);
imshow(INNZoom)
title('Step-5.1: Zoom in Nearest Neighbour');
IBIZoom = Zoomin(IBI, 900,600,1200,900);
subplot(1,2,2);
imshow(IBIZoom)
title('Step-5.2: Zoom in Bilinear interpolation')


function im = Zoomin(Imgzoom, TLX, TLY, BRX, BRY)
%Pulling in an Image to zoom into using coordinates
%TRX = Top Left X, TRY = Top Left Y, 
%BRX = Bottom Right X, BRY = Bottom Right Y
sizex = BRX - TLX;
sizey = BRY - TLY;
im = uint8(zeros(sizex,sizey));
ROW = 0; COL = 0;
    for row = TLX:(BRX - 1)
        ROW = ROW + 1;
        for col = TLY:(BRY - 1)
            COL = COL + 1;
            im(COL, ROW) = Imgzoom(col, row);
        end
        COL = 0;
    end
end

%References:
% https://www.theengineeringprojects.com/2016/02/image-zooming-bilinear-interpolation-matlab.html
% http://supercomputingblog.com/graphics/coding-bilinear-interpolation/
% https://rosettacode.org/wiki/Bilinear_interpolation
% https://www.youtube.com/watch?v=AqscP7rc8_M
% https://www.tutorialspoint.com/matlab/matlab_functions.htm
% https://www.mathworks.com/help/vision/ug/interpolation-methods.html
