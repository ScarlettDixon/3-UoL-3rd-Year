function[] = Work3a()

%Convert to gray
lennagray = rgb2gray(Lenna);
imshow(lennagray)

%Transpose gray img
lennagray = rgb2gray(Lenna);
lengrayrot = lennagray'
imshow(lengrayrot)
%or
lennagray = rgb2gray(Lenna);
A = uint8(zeros(512,512));
A = lennagray;
B = uint8(zeros(512,512));
for row = 1:size(A,1)
 for col = 1:size(A,2)
	B(col,row) = A(row,col);
 end
end
imshow(B)


%Flip gray img vertically
lennagray = rgb2gray(Lenna);
lengrayflip = fliplr(lennagray)
imshow(lengrayflip)
%or
lennagray = rgb2gray(Lenna);
A = uint8(zeros(512,512));
A = lennagray;
B = uint8(zeros(512,512));
for row = 1:size(A,1)
 for col = 1:size(A,2)
	B(row, size(A,1) - (col-1)) = A(row,col);
 end
end
imshow(B)


%Zoom gray img
lennagray = rgb2gray(Lenna);
A = uint8(zeros(512,512));
A = lennagray;
B = uint8(zeros(150,150));
ROW = 0; COL = 0;
for row = 250:400
    ROW = ROW + 1;
 for col = 250:400
	COL = COL + 1;
     B(ROW,COL) = A(row,col);
 end
 COL = 0; %Key part reset columns once new row
end
imshow(B)

%Convert to grayscale then reduce number the variance of greyscale - colourbar
lennagray = rgb2gray(Lenna);
A = uint8(zeros(512,512));
A = lennagray;
B = uint8(zeros(512,512));
for row = 1:size(A,1)
 for col = 1:size(A,2)
	B(row,col) = uint8(floor(A(row,col)/8));
 end
end
imshow(B)
%colorbar;

lennagray = rgb2gray(Lenna);
A = uint8(zeros(512,512));
A = lennagray;
B = uint8(zeros(512,512));
for row = 1:size(A,1)
 for col = 1:size(A,2)
	B(row,col) = 32*(uint8(floor(A(row,col)/32)));
 end
end
imshow(B)
colorbar;