function xx = smoother(x, number_of_frame, NE)
% This file gets the precise points along the analytical piecewise
% Lagrange functions

[~,n] = size(x);
xx = zeros(number_of_frame + 1,n);
for ii = 1:n
    basic = x(:,ii);
    xx(:,ii) = print_lagrange(basic, number_of_frame,NE);
end