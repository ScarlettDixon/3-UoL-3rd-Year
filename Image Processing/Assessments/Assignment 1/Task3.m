% MATLAB script for Assessment Item-1
% Task-3
% Functions not allowed: None
% Notes:
% Imerode, Imclose, ImOpen, Imdilate important
% bwboundaries and regionprops may be useful
% Star Equation: A = (1/2) (b) [ (b/2) / tan(B/2) ] = b2 / [ 4 tan (B/2) ]
% Have Boundary of each item us bwboundary
% Have Centre point from regionprops
% Use B to find x and Y values of the boundaries of objects
% Calculate the distance using the centroid
% Use findpeaks to find peaks
clear; close all; clc;

% Step-1: Load input image
K = imread('Starfish.jpg'); % Size = 556x612
subplot(2,3,1);
imshow(K);
title('Step-1: Load input image');

% Step-2: Remove Noise from Image
KNoNoise = K;
% KNoNoise2 = K;
% KNoNoise = padarray(KNoNoise,[2 2]); %Padd with Two lines of zeros
% i = size(KNoNoise,1)-2;
% j = size(KNoNoise,2)-2;
% Table = uint8(zeros(3,25));
% count = 1;
% for rgb = 1:3
%     for row = 3:i
%         for col = 3:j
%             for countrow = -2:2
%                 for countcol = -2:2
%                     Table(1, count) = KNoNoise((row + countrow), (col + countcol), rgb);
%                     count = count +1;
%                 end
%             end
%             Table = sort(Table);
%         	count = 1;
%             KNoNoise2(row-2,col-2,rgb) = Table(rgb,13); %
%         end
%     end
% end
% KNoNoise2 = medfilt3(KNoNoise);
% subplot(2,3,2);
% imshow(KNoNoise2);
% title('Step-2: Remove Noise from Image');

% Step-3: Convert To Grey 
KGray = rgb2gray(K);%NoNoise2);
subplot(2,3,2);
% imshow(KGray);
% title('Step-3: Convert To Grey');

% Step-3.5: Remove Noise after gray conversion
% Filtering after conversion to grey allows for less values to occur,
% reducing noise further
imshow(KGray);
title('Step-2: Convert To Grey')

% Step-3: Remove Noise after gray conversion
KGray = medfilt2(KGray);
subplot(2,3,3);
imshow(KGray);
title('Step-3: Remove Noise after gray conversion')

% Step-4 Filter through Histograms
KHist = histeq(KGray, 3); %2 is not enough, 4 too much
subplot(2,3,4);
imshow(KHist);
title('Step-4 Filter through Histograms');

% Step-5: Convert To Binary 
KBin = imbinarize(KHist);
KBin = ~ KBin; %S
subplot(2,3,5);
imshow(KBin);
title('Step-4: Convert To Binary');

% Test-1: Search after manipulation
se = strel('disk', 3);
setest = strel('disk', 1);
imdi = imdilate(KBin, se);
imer = imerode(KBin, se);
imop = imopen (KBin, se);
imcl = imclose(KBin, se);
BoEx = KBin - imer; 
ExConCom = imdi + KBin;
imopcl = imclose(imop, se);
imoper = imerode(imop, setest);
KBin = imoper;
% ReFill = imfill(KBin);
% 
% subplot (2,4,1);
% imshow(imdi);

% title('Imdilate Image');
% subplot (2,4,2);
% imshow(imer);
% title('Imerode Image');
% subplot (2,4,3);
% imshow (imop);
% title('Imopen Image');
% subplot (2,4,4);
% imshow(imcl);
% title('Imclose Image');
% subplot (2,4,5);
% imshow(BoEx);
% title('Boundary Extraction');
% subplot (2,4,6);
% imshow(ExConCom);
% title('Extracting Connected Components');
% subplot (2,4,7);
imshow (imoper);
%title('Testing: Open then Erode');
title('Step-6: Open then Erode');
% Test-2: Search using bwboundaries
subplot(2,3,6);
[B,L, n, A] = bwboundaries(KBin,'noholes');
%imshow(L);
% imshow(label2rgb(L, @jet, [.5 .5 .5]))
% hold on
% for k = 1:length(B)
%    boundary = B{k};
%    plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
% end

%Test-3: Search By RGB Colour
% KColour = KNoNoise;
% KGray2 = rgb2gray(KNoNoise);
% i = size(KColour,1);
% j = size(KColour,2);
% for row = 1:i
%     for col = 1:j
%         if (KColour(row, col, 1) > 229 && KColour(row, col, 1) < 255) && (KColour(row, col, 2) > 169 && KColour(row, col, 2) < 211) &&(KColour(row, col, 3) > 77 && KColour(row, col, 3) < 170)
%             KGray2(row, col) = 255;
%         else
%             KGray2(row, col) = 0;
%         end
%     end
% end
% subplot(2,4,5);
% imshow(KGray2)
% title('Test-3: Search By RGB Colour');

% Test-4: Search using regionprops
Kstats  = regionprops('table', KBin, 'Centroid', 'Area', 'Perimeter', 'PixelList');
KCent = Kstats.Centroid; %Centre point of the object
KArea = Kstats.Area; %Area of the object
KPixel = Kstats.PixelList; %All pixels within object
% subplot(2,4,7);
% imshow (KBin);
% hold on
% for bound = 1:length(B)
% %     radiusx = Kcent(bound);
% %     radiusy = Kcent(bound);
%     Boundary = B{bound};
%     plot(Boundary(:,2), Boundary(:,1), 'b', 'LineWidth', 2)
% end
 KStar = KBin;
% Bleng = length(B); %Number of boundaries in the image
% Count = 1;
 for bound = 1:length(KArea) %Loop though all Areas
     BTarget = B{bound}; %Which Area array will be being used
     KAreaTarg = KArea(bound); %The area of the item in question
     KPixTarget = KPixel{bound}; 
     if KAreaTarg < 1114 || KAreaTarg > 1352 || KArea(bound) == 1206
         for X = 1:length(KPixTarget)
            KStar(KPixTarget(X,2), KPixTarget(X,1)) = 0;
         end
     else
         %Ar = KArea(bound)
     end
 end

imshow(KStar);
title('Final Image');

% References:
% https://www.mathworks.com/discovery/image-segmentation.html
% https://www.mathworks.com/help/images/ref/histeq.html
% https://www.mathworks.com/matlabcentral/answers/116793-how-to-classify-shapes-of-this-image-as-square-rectangle-triangle-and-circle
% https://www.mathworks.com/help/images/ref/regionprops.html
% http://mathcentral.uregina.ca/QQ/database/QQ.09.06/s/chetna2.html