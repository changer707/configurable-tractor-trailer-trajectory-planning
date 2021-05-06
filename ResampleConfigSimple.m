function xx = ResampleConfigSimple(x,NE)
[Nv, Nfe] = size(x);
xx = [];
LARGE_NUM = 100;
for ii = 1 : Nfe
    temp_x = [];
    for jj = 1 : (Nv-1)
        temp = linspace(x(jj,ii), x(jj+1,ii), LARGE_NUM);
        temp_x = [temp_x, temp(1,1:(LARGE_NUM - 1))];
    end
    temp_x = [temp_x, x(Nv,ii)];
    index = round(linspace(1,length(temp_x),NE));
    xx = [xx; temp_x(index)]';
end 
end