%Workshop 7
% """[img, map] = imread('coins.gif','frames','all');
% img2=imread('noisy-fingerprint.tif');
% se1=strel('disk',1);
% se2=strel('disk',10);
% close =imerode(img,se2);
% B1=strel([0 0 0; 0 1 1; 0 1 1]);
% B2=strel([1 1 1; 1 0 0; 1 0 0]);
% g=bwhitmiss(shapes,B1,B2);
% img2e = imerode(img2,se1);
% [L, num1] = bwlabel(img2e, 8);
% lbl = label2rgb(L, 'jet', 'k', 'shuffle');
% [M, num2] = bwlabel(close, 8);
% lbl2 = label2rgb(M, 'jet','k','shuffle');
% imshow(lbl2);"""

% Task1
%b = imread('Image Input');
b = imread('noisy-fingerprint.tif');
%se = strel(shape, parameters);
se = strel('disk', 2);
imdi = imdilate(b, se);
imer = imerode(b, se);
imop = imopen (b, se);
imcl = imclose(b, se);

subplot (2,3,1);
imshow(b);
title('Input Image');
subplot (2,3,2);
imshow(imdi);
title('Imdilate Image');
subplot (2,3,3);
imshow(imer);
title('Imerode Image');
subplot (2,3,4);
imshow(imop);
title('Imopen Image');
subplot (2,3,5);
imshow(imcl);
title('Imclose Image');

%Task 2
%boundary extraction: Original Image - Erode
%extracting connected components:
%region filling:
b = imread('coins.gif');
%imwrite(b, 'coins.tif');
%b = imread('coins.tif');
se = strel('disk', 3);
imdi = imdilate(b, se);
imer = imerode(b, se);
imop = imopen (b, se);
imcl = imclose(b, se);
BoEx = b - imer; 
ExConCom = imdi + b;
ReFill = imfill(b);

subplot (3,3,1);
imagesc(b);
title('Input Image');
subplot (3,3,2);
imagesc(imdi);
title('Imdilate Image');
subplot (3,3,3);
imagesc(imer);
title('Imerode Image');
subplot (3,3,4);
imagesc(imop);
title('Imopen Image');
subplot (3,3,5);
imagesc(imcl);
title('Imclose Image');
subplot (3,3,6);
imagesc(BoEx);
title('Boundary Extraction');
subplot (3,3,7);
imagesc(ExConCom);
title('Extracting Connected Components');
subplot (3,3,8);
imagesc(ReFill);
title('Region Filling');