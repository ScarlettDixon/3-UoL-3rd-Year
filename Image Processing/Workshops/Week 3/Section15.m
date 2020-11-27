function[] = Section15()

x = int32([5.32 3.47 6.28]) .* 7.5
x = int64([5.32 3.47 6.28]) .* 7.5
x = num2cell(x)


% displaying the smallest and largest single-precision
% floating point number
str = 'The range for single is:\n\t%g to %g and\n\t %g to %g';
sprintf(str, -realmax('single'), -realmin('single'), ...
 realmin('single'), realmax('single'))
% displaying the smallest and largest double-precision
% floating point number
str = 'The range for double is:\n\t%g to %g and\n\t %g to %g';
sprintf(str, -realmax('double'), -realmin('double'), ...
 realmin('double'), realmax('double'))