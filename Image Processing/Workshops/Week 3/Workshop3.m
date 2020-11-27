function[] = Work3a()

%Black background, white circle
A = uint8(zeros(200,200));
radius = 100;
for row = 1:size(A,1)
 for col = 1:size(A,2)
	if ( sqrt( (row - size(A,1)/2)^2 + ( col - size(A,1)/2 )^2 ) < radius )
		A(row,col) = 255;
	end
 end
end
imshow(A)

%White  background, black circle
A = uint8(zeros(200,200));
radius = 100;
for row = 1:size(A,1)
 for col = 1:size(A,2)
	if ( sqrt( (row - size(A,1)/2)^2 + ( col - size(A,1)/2 )^2 ) < radius )
		A(row,col) = 0;
	else
		A(row,col) = 255;
	end
 end
end
imshow(A)

%Black background, gradient column circle
A = uint8(zeros(200,200));
b = 255/100;
radius = 100;
for row = 1:size(A,1)
 for col = 1:size(A,2)
	if ( sqrt( (row - size(A,1)/2)^2 + ( col - size(A,1)/2 )^2 ) < radius )
		A(row,col) = col * b;
	end
 end
end
imshow(A)

%White  background, gradient column circle
A = uint8(zeros(200,200));
b = 255/100;
radius = 100;
for row = 1:size(A,1)
 for col = 1:size(A,2)
	if ( sqrt( (row - size(A,1)/2)^2 + ( col - size(A,1)/2 )^2 ) < radius )
		A(row,col) = col * b;
	else
		A(row,col) = 255;
	end
 end
end
imshow(A)

%Black background, gradient row circle
A = uint8(zeros(200,200));
b = 255/100;
radius = 100;
for row = 1:size(A,1)
 for col = 1:size(A,2)
	if ( sqrt( (row - size(A,1)/2)^2 + ( col - size(A,1)/2 )^2 ) < radius )
		A(row,col) = row * b;
	end
 end
end
imshow(A)

%Black background, gradient column circle, 80 radius
A = uint8(zeros(200,200));
b = 255/80;
radius = 80;
for row = 1:size(A,1)
 for col = 1:size(A,2)
	if ( sqrt( (row - size(A,1)/2)^2 + ( col - size(A,1)/2 )^2 ) < radius )
		A(row,col) = col * b;
	end
 end
end
imshow(A)