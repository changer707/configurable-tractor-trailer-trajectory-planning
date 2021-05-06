function [X,Y]=Bazier(x,y,NE)
n=length(x);
t=linspace(0,1,NE);
xx=0;yy=0;
for k=0:n-1
    tmp=nchoosek(n-1,k)*t.^k.*(1-t).^(n-1-k);
    xx=xx+tmp*x(k+1);
    yy=yy+tmp*y(k+1);
end
    X=xx;Y=yy;
end