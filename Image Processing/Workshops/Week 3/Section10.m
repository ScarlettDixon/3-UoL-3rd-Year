function[] = Section10()

a = 10;
% while loop execution
while( a < 20 )
 fprintf('value of a: %d\n', a);
 a = a + 1;
end

for a = 10:20
 fprintf('value of a: %d\n', a);
end

for a = 1.0: -0.1: 0.0
 disp(a)
end

for a = [24,18,17,23,28]
 disp(a)
end

" Nested For loop syntax
for m = 1:j
 for n = 1:k
 <statements>;
 end
end"

"Nested While loop
while <expression1>
 while <expression2>
 <statements>
 end
end"

for i=2:100
 for j=2:100
 if(~mod(i,j))
 break; % if factor found, not prime
 end
 end
 if(j > (i/j))
 fprintf('%d is prime\n', i);
 end
end

v = [1: 2: 20];
sv = v.* v; %the vector with elements
 % as square of v's elements
dp = sum(sv); % sum of squares -- the dot product
mag = sqrt(dp); % magnitude
disp('Magnitude:'); disp(mag);

v1 = [2 3 4];
v2 = [1 2 3];
dp = dot(v1, v2);
disp('Dot Product:'); disp(dp);

v = [1: 2: 20];
sqv = v.^2;
disp(v);disp(sqv);
