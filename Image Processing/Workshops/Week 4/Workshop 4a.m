
%Histograms
imshow(small);
colorbar;
colormap gray;

imagesc(small);
colorbar;
colormap gray;

I = small;
mi = min(min(I));
ma = max(max(I));
L = ma - mi + 1;
for i = 1:L
 pixel_value(i) = i - 1;
 frequency = find( I == pixel_value(i) );
 Nk(i) = length( frequency );
end
bar(pixel_value,Nk,0.1);

%Contrast Stretching

%linear contrast stretching
P = imread('Pout.png');
I = rgb2gray(P);
J = 255*im2double(I); % converts the intensity image I to double precision
mi = min(min(J)); % find the minimum pixel intensity
ma = max(max(J)); % find the maximum pixel intensity
J1 = zeros(size (I));
for row = 1 :size(J,1)
 for col = 1:size(J,2)
    J1(row,col) = (255 * (J(row,col)- mi))/(ma - mi);
 end
end
J1 = im2uint8(J1/255); % converts the grayscale image I to uint8
%imshow(J1);
%J2 = imadjust(I,[low_in; high_in],[low_out; high_out]);
J2 = imadjust(I,[mi/255; ma/255],[0; 1]); %The above euqtion but simpler
Z = imsubtract(J1,J2); % Subtract the two images, should be black
%imshow (Z);
%imhist(I,n) where n is number of bins (intensity levels)
imhist(I,8) %Find out L, ma - mi + 1

%Histogram Equalisation
%Task 4.1
I = imread('small.png');
I = 255*im2double(I);
mi = min(min(I));
ma = max(max(I));
L = ma - mi + 1;
for i = 1:L
 pixel_value(i) = i - 1; %0 to 7
 frequency = find( I == pixel_value(i) ); %How much of each pixel value exists in the picture
 Nk(i) = length( frequency ); %h(k) = nk, k is the sum of all values between 0 and L-1, nk is the number of pixels with intensity k 
 pk(i) = (Nk(i) / (50 * 50)); %p(k) = (nk / MN) where M and N are the size of the image
 if i == 1 % no previous variable
  pj(i) = pk(i); 
 else % add sum so far to the new variable
  pj(i) = pk(i) + pj (i-1);   
 end
 Tk(i) = ((L-1) * pj(i));
 sk(i) = round(Tk(i), 0 );
end
%Nk = 135 473 626 824 109 108 106 119
%pr(rk) = 0.0540 0.1892 0.2504 0.3296 0.0436 0.0432 0.0424 0.0476
%Sum (pr(rj)) = 0.054 0.243 0.4396 0.8232 0.8668 0.9100 0.9524 1.0000
%T(rk) = 0.3780 1.7024 3.4552 5.7624 6.0676 6.3700 6.6668 7 
%sk = 0 2 3 6 6 6 7 7

%Output Image
J = uint8(zeros(size(I)));
for row = 1 :size(I,1)
 for col = 1:size(I,2)
 J(row,col) = sk(I(row,col)+1);
 end
end
for i = 1:L
 pixel_value_en(i) = i - 1;
 frequency_en = find( J == pixel_value_en(i) );
 Nk_en(i) = length( frequency_en );
end
subplot(2, 2, 1) % Three number, Number of Rows, then columns, then linear index
imagesc(I);
colorbar;
colormap gray;
subplot(2, 2, 2)
imagesc(J);
colorbar;
colormap gray;
subplot(2, 2, 3)
bar(pixel_value,Nk,0.1);
subplot(2, 2, 4)
bar(pixel_value_en, Nk_en, 0.1);
%imhist(pixel_value_en,8);

%Task 4.2
H = imread('Pout.png');
I = rgb2gray(H);
J = histeq(I,256);
subplot(2, 2, 1) % Three number, Number of Rows, then columns, then linear index
imshow(I);
subplot(2, 2, 2)
imshow(J);
subplot(2, 2, 3)
imhist(I,256);
subplot(2, 2, 4)
imhist(J,256);