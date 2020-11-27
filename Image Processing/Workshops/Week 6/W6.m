%Sharpening Spatial Filters

%Task 2.1
I=imread('Pout.png');
I_gray=rgb2gray(I);
C=im2double(I_gray);
Gx=zeros(size(C)); % create an empty image with the same size as the input image
Gy=zeros(size(C));
Gf=zeros(size(C));

%x = [-1 0 1
%     -2 0 2
%     -1 0 1];
%y = [ 1 2 1
%      0 0 0
%    -1 -2 -1];
%ij= [(i-1,j-1) (i-1,j) (i-1,j+1)
%      (i,j-1)   (i,j)   (i,j+1)
%	  (i+1,j-1) (i+1,j) (i+1,j+1)


for i = 2:size(C,1) - 1 %Start from second row, second column, so edges are caught in 
 for j = 2:size(C,2) - 1
    Gx(i,j) = C(i-1, j+1) + C(i+1, j+1) + -C(i-1, j-1) + -C(i+1,j-1) + (2 * C(i, j+1) - C(i, j-1));  %Miss middle column, (i,j), (i-1,j), (i+1,j) 
	Gy(i,j) = C(i-1, j-1) - C(i+1,j-1) + C(i-1, j+1) - C(i+1, j+1) + (2 * C(i-1, j) - C(i+1, j));%Miss middle row, (i,j), (i,j-1), (i, j+ 1)
	Gf(i,j) = sqrt ((Gx(i,j)^2) + (Gy(i,j)^2));
 end
end

subplot(2,2,1);
imshow(C);
title('Input Image');
subplot(2,2,2);
imshow(Gx);
title('Vertical Edge Image');
subplot(2,2,3);
imshow(Gy);
title('Horizontal Edge Image');
subplot(2,2,4);
imshow(Gf);
title('Final Edge Image');

%Task 2.2
I=imread('Pout.png');
I_gray=rgb2gray(I);
C=im2double(I_gray);
Gx=zeros(size(C)); 
Gy=zeros(size(C));
Gf=zeros(size(C));
X = fspecial('sobel')';
Y = fspecial('sobel');
Gx = imfilter(C, X);
Gy = imfilter(C, Y);
%Gf = imfilter (C, sqrt (((Gx)^2) + ((Gy)^2)));
subplot(2,2,1);
imshow(C);
title('Input Image');
subplot(2,2,2);
imshow(Gx);
title('Vertical Edge Image');
subplot(2,2,3);
imshow(Gy);
title('Horizontal Edge Image');
subplot(2,2,4);
imshow(Gf);
title('Final Edge Image');