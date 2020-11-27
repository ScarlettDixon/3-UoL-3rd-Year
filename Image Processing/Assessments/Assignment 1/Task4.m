% MATLAB script for Assessment Item-1
% Task-4
% Functions not allowed: None
% Notes:
clear; close all; clc;

% Step-1: Load input image
K = imread('Starfish.jpg'); % Size = 556x612
subplot(2,3,1);
imshow(K);
title('Step-1: Load input image');

% Step-2: Convert To Grey 
KGray = rgb2gray(K);
subplot(2,3,2);
imshow(KGray);
title('Step-2: Convert To Grey');

% Step-3: Remove Noise after gray conversion
% Filtering after conversion to grey allows for less values to occur,
% reducing noise further
KGray = medfilt2(KGray);
subplot(2,3,3);
imshow(KGray);
title('Step-3: Remove Noise after gray conversion')

% Step-4 Filter through Histograms
% Using Histograms allows the sharpening of the Image
KHist = histeq(KGray, 3); %2 is not enough, 4 too much
subplot(2,3,4);
imshow(KHist);
title('Step-4 Filter through Histograms');

% Step-5: Convert To Binary 
KBin = imbinarize(KHist);
KBin = ~ KBin; %Inverse the Binary for black background with white shapes
subplot(2,3,5);
imshow(KBin);
title('Step-5: Convert To Binary');

% Step-6: Open then Erode
%Figured out the best method in previous task
seop = strel('disk', 3);
seer = strel('disk', 1);
imop = imopen (KBin, seop);
imoper = imerode(imop, seer);
KBin = imoper;
imshow (imoper);
title('Step-6: Open then Erode');

% Step-7 Find Boundaries and Centre
subplot(2,3,6);
[B,L, n, A] = bwboundaries(KBin,'noholes');
%imshow(L);

Kstats  = regionprops('table', KBin, 'Centroid', 'Area', 'Perimeter', 'PixelList');
KCent = Kstats.Centroid; %Centre point of the object
KArea = Kstats.Area; %Area of the object
KPixel = Kstats.PixelList; %All pixels within object

% Step-8 Find Peaks and Troughs
KStar = KBin;
Bleng = length(B); %Number of boundaries in the image
Count = 1;
for bound = 1:length(B) %Loop though all Boundaries
    BounLeng = length(B{bound}); %Number of pixels on the boundary
    Centx = KCent(bound, 1); % X-value of the centre of mass
    Centy = KCent(bound, 2); % Y-value of the centre of mass
    BTarget = B{bound}; %Which Boundary array will be being used
    Bdist = zeros(length(BounLeng));
    for xy = 1:BounLeng %Go through every pixel on the boundary
        Bx = BTarget(xy, 1);
        By = BTarget(xy, 2);
        Radx = Centx - Bx;
        Rady = Centy - By;
        Disp = sqrt(Radx^2 + Rady^2);
        Bdist(xy) = Disp;
    end
    %plot(Bdist);
    numPeak = numel(findpeaks(Bdist, 'MinPeakProminence', 0.6, 'MinPeakWidth', 2.104 ));
    %promPeak = numel((findpeaks(Bdist, 'MinPeakProminence', 0.6)));
    %widPeak = numel((findpeaks(Bdist, 'MinPeakWidth', 2.104)));
    %RevBdist = max(Bdist) - Bdist;
    %numtrough = numel(findpeaks(-Bdist));
    %&& widPeak ~= 5 && promPeak ~= 6 
    KPixTarget = KPixel{bound}; 
     if numPeak ~= 3 %If there are not 5 points to the star, remove it from Image
        for X = 1:length(KPixTarget)
            KStar(KPixTarget(X,2), KPixTarget(X,1)) = 0;
        end
    else
        % numuse = promPeak;
        % numuse2 = numtrough
        % staruse = bound
        % [~, ~, w, p] = findpeaks(Bdist, 'MinPeakProminence', 0.6,'MinPeakWidth', 2.104 )
        % Ar = KArea(bound);
    end
%     if numpeak > 4
%         numuse = numpeak;
%     end
%     plot(Bdist(:,1) , Bdist(:,2));
%     num = findpeaks(Bdist(:,1) , Bdist(:,2));
        %Commented out below is the code to look at the graphs from the report
% %     subwidth = 8; 
% %     subheight = 9;
% %     subplot(subwidth, subheight, Count);
% %     findpeaks(Bdist, 'MinPeakProminence', 0.6, 'MinPeakWidth', 2.104);
% %     %plot(Bdist);
% %     %title(sprintf('%f' ,Count));
% %     if numPeak == 5
% %         title([num2str(Count),' - Star']);
% %     else
% %         title(Count);
% %     end
% %     Count = Count + 1;
end
subplot(2,3,6);
imshow(KStar);
title('Final Image');