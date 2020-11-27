
%Spatial Filtering

%Task 2.1
%For loop version
x = uint8(10 * magic(5))
x = 255*im2double(x);
sum = 0;
for row = 1:3
 for col = 1:3
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 1:3
 for col = 2:4
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 1:3
 for col = 3:5
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 2:4
 for col = 1:3
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 2:4
 for col = 2:4
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 2:4
 for col = 3:5
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 3:5
 for col = 1:3
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 3:5
 for col = 2:4
 sum = sum + (1/9)*x(row,col);
 end
end
sum = 0;
for row = 3:5
 for col = 3:5
 sum = sum + (1/9)*x(row,col);
 end
end

%Mean2 method
mean2(x(1:3,1:3));
mean2(x(1:3,2:4));
mean2(x(1:3,3:5)); 
mean2(x(2:4,1:3)); 
mean2(x(2:4,2:4)); 
mean2(x(2:4,3:5)); 
mean2(x(3:5,1:3)); 
mean2(x(3:5,2:4)); 
mean2(x(3:5,3:5));  

%Shorter nested with mean2
for row = 1:3
	for col = 1:3
	mean2(x(row:(row+2), col: (col+2)))
	end
end
%Shorter nested with mean2 for any size matrix
for row = 1:(size(x,1) - 2)
	for col = 1:(size(x,2) - 2)
	mean2(x(row:(row+2), col: (col+2)))
	end
end
%Shorter nested with for loop
sum = 0;
for row = 1:(size(x,1) - 2)
	for col = 1:(size(x,2) - 2)
		sum = sum + (1/9)*x(row:(row+2), col:(col+2));
	end
end
sum = round(sum,0)

%Task 2.2
%Edge Padding
x = uint8(10 * magic(5));
x = 255*im2double(x);
x	= padarray(x,[1 1]);
sum = 0;
for row = 1:3	% Size of mask, can be increased to 1:5 for 5x5 filter
	for col = 1:3
		sum = sum + (1/9)*x(row:(row+4), col:(col+4)); %Calculates
	end
end
sum = round(sum,0)

%for loop that works for any size
x = uint8(10 * magic(5));
x = 255*im2double(x);
x	= padarray(x,[1 1]);
sum = 0;
i = size(x,1) - 3;
j = size(x,2) - 3;
mask_nxn = 3;
for row = 1:mask_nxn
	for col = 1:mask_nxn
		sum = sum + (1/9)*x(row:(row+i), col:(col+j));
	end
end
sum = round(sum, 0)

%Smoothing Spatial filters

%Task 3.1
A	= uint8([3 6 7 4; 0 6 9 8; 1 45 8 18; 2 1 7 9]);
A = 255*im2double(A);
A	= padarray(A,[1 1]);
sum = 0;
i = size(A,1) - 3;
j = size(A,2) - 3;
mask_nxn = 3;
for row = 1:mask_nxn
	for col = 1:mask_nxn
		sum = sum + (1/9)*A(row:(row+i), col:(col+j));
	end
end
sum = round(sum, 0)

%Task 3.2
InputImage = imread('Test.tif');
if ( length(size(InputImage))> 2 )
 InputImage = rgb2gray(InputImage);
end
figure;
subplot(2,2,1);
imshow(InputImage);
title('Input Image');
h = fspecial('average', 3);
FilteredImage = imfilter(InputImage,h);
subplot(2,2,2);
imshow(FilteredImage);
title('Filtered Image 3×3');
h = fspecial('average', 5);
FilteredImage = imfilter(InputImage,h);
subplot(2,2,3);
imshow(FilteredImage);
title('Filtered Image 5×5');
h = fspecial('average', 7);
FilteredImage = imfilter(InputImage,h);
subplot(2,2,4);
imshow(FilteredImage);
title('Filtered Image 7×7');

%Task 3.3/4
B	= uint8([3 6 7 4; 0 6 9 8; 1 45 8 18; 2 1 7 9]);
B	=	medfilt2 (B,[3 3], 'zeros')

I = imread('Test.tif');
J	=	medfilt2 (I,[3 3], 'zeros');
subplot(1,2,1);
imshow(I);
subplot(1,2,2);
imshow(J);
