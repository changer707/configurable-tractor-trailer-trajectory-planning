function X=Bspline(path)
P=path';
n = size(P);
n=n(2);
knots = aptknt(linspace(0,1,n),3);%生成B样条曲线节点
sp = spmak(knots,P);%生成B样条函数
X=fnplt(sp,[knots(1),knots(length(knots))]);%根据所有节点，画样条曲线图
X=X';
end