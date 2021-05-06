% A*轨迹规划+曲线二次规划
start = [x0_1,y0_1];
goal = [x_center_tf,y_center_tf];
globalpath = Astar(start,goal); %A*原始轨迹
g_x = ResampleConfigSimple(globalpath(:,1),NE);  %重新采样x
g_y = ResampleConfigSimple(globalpath(:,2),NE);   %y
plot(g_x,g_y,'b')
hold on
gpath = [g_x,g_y];
% B样条
% Bsplinepath = Bspline(gpath);
% s_x = ResampleConfigSimple(Bsplinepath(:,1),NE);
% s_y = ResampleConfigSimple(Bsplinepath(:,2),NE);
% Bsplinepath = [s_x,s_y]; %样条曲线拟合后
% plot(s_x,s_y,'r')
% scatter(s_x,s_y,'*')
% Bazier曲线
[b_x,b_y] = Bazier(g_x,g_y,NE);
Bazierpath = [b_x;b_y]'; %Bazier曲线拟合后
plot(Bazierpath(:,1),Bazierpath(:,2),'g')
scatter(Bazierpath(:,1),Bazierpath(:,2),'*')


