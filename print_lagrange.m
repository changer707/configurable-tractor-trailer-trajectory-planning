function out = print_lagrange(basic,number_of_frame,NE)
% Generate the sample points on each pircewise Larange intervals according
% to the given collocation points.

out = [];
for ii = 1 : NE
    data = basic(((ii-1)*4+1):(ii*4));
    tau = [0, 0.1550510257216822, 0.6449489742783178, 1]; % Collocation point related parameters: Tau ( K = 3 by default)
    ym = linspace(0,1,(number_of_frame./NE+1));
    out = [out,lagrange(tau, data,ym,ii,NE)];
end