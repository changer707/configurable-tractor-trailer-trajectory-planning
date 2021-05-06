
function  globalpath = Astar(start,goal)
map.grid_length = 1;%栅格边长
map.start=start;%起点
map.goal=GetGoal(start,map.grid_length,goal);%终点
global  polygon_obstacle_vertex
obstacle =  polygon_obstacle_vertex;
obstacle = GetObs(obstacle,start,map.grid_length); %n*2

open=[];
close=[];
globalpath=[];
findflag=false; %判断是否找到路径

%open=[x,y,F,G,parentx,parenty]
open(1,:)=[map.start(1),map.start(2),0+h(map.start,map.goal),0,map.start(1),map.start(2)];
next=MOTIONMODEL(map);

while ~findflag
    if isempty(open(:,1))
       % findflag=true;
        disp('no path!');
        return;
    end
    
    [inopenflag,inline]=ISINOPEN(map.goal,open);%判断目标点是否在open中
    if inopenflag
        findflag=true;
        disp('find goal');
        close=[open(inline,:);close];%把找到的open行放入close中
        break;
    end
    
    [B,I]=sort(open(:,3));%升序
    open=open(I,:);%把open也做F的升序排列
    
    close=[open(1,:);close];%把F最小的点从open放入close
    current=close(1,:);
    open(1,:)=[];%放入close的点要及时在open中移除
    
    %依次对每个相邻节点做处理
    for i=1:length(next(:,1))
        m=[current(1)+next(i,1),current(2)+next(i,2),0,0,0,0];
        m(4)=current(4)+next(i,3);%G
        m(3)=m(4)+h(m(1:2),map.goal);%F
        
        if ISOBSTACLE(m,obstacle)
            continue;
        end
        
        %typeflag==1,在close中；
        %typeflag==2,不在open中；
        %typeflag==3,已经在open中；
        [typeflag,inc]=FINDTYPE(m,open,close);
        
        if typeflag==1  %在close中
            continue;
        
        elseif typeflag==2 %不在open中
            m(5:6)=[current(1),current(2)];
            open=[open;m];
        
        else  %已经在open中
           if m(3)<open(inc,3)  %若当前节点作为父节点更好，则变更父节点
               m(5:6)=[current(1),current(2)];  
               open(inc,:)=m; %%%%%%更新open中的对应行的父节点，因为实际筛选当前节点是在open中选出
           end
        end     
    end
end
    globalpath = GETPATH( close , map.start );
    globalpath = rot90(globalpath,2);
    globalpath = fliplr(globalpath);
end

function obs = GetObs(obstacle,start,grid_length)
p = [];obs = [];
p(1,:) =start(1)-30:grid_length:start(1)+30;
p(2,:) =start(2)-30:grid_length:start(2)+30;
for i = 1:length(p(1,:))
    for j = 1:length(p(2,:))
        pp = [p(1,i),p(2,j)];
        if IsInObs(pp,obstacle)
            obs = [obs;pp];
        end
    end
end
end
%判断点p是否在所有四边形内
function flag = IsInObs(p,obstacle)
flag = false;obs=[];
if length(obstacle)>8
    num = length(obstacle)/8;
    obs = cell(1,num);
    for i = 1:num
        obs{i} = obstacle(1+8*(i-1):8*i);
    end
else
    num=1;obs = cell(1,num);
    for i = 1:num
        obs{i} = obstacle(1+8*(i-1):8*i);
    end
end
for i = 1:num
    flag = isinrec(obs{i},p);
    if flag
        break;
    end
end
end
% 判断点p是否在四边形内
function flag = isinrec(obs,P)
A = obs(1:2);B = obs(3:4);C = obs(5:6);D = obs(7:8);
AB = [B(1)-A(1),B(2)-A(2)];BC = [C(1)-B(1),C(2)-B(2)];CD = [D(1)-C(1),D(2)-C(2)];DA = [A(1)-D(1),A(2)-D(2)];
AP = -[A(1)-P(1),A(2)-P(2)];BP = -[B(1)-P(1),B(2)-P(2)];CP = -[C(1)-P(1),C(2)-P(2)];DP =  -[D(1)-P(1),D(2)-P(2)];
eq1 = Cross(AP,AB);
eq2 = Cross(BP,BC);
eq3 = Cross(CP,CD);
eq4 = Cross(DP,DA);
if eq1>0&&eq2>0&&eq3>0&&eq4>0
    flag = true;%在内部
else
    flag = false;
end
end
function v = Cross(A,B)
v = A(1)*B(2)-A(2)*B(1);
end
function path=GETPATH(close,start)

ind=1;
path=[];
while 1
    path=[path; close(ind,1:2)];
    if isequal(close(ind,1:2),start)   
        break;
    end
    for io=1:length(close(:,1))
        if isequal(close(io,1:2),close(ind,5:6))
            ind=io;
            break;
        end
    end
end
end

% 默认起点在栅格中心，将终点修正到栅格中心
function goal_fix = GetGoal(start,grid_length,goal)
m = goal(1)-start(1);
n = goal(2)-start(2);
if m>0 && n>0
    d_x = ceil(m/grid_length);
    d_y = ceil(n/grid_length);
elseif m>0 &&n<0
    d_x = ceil(m/grid_length);
    d_y = ceil(n/grid_length)-1;
elseif m<0&&n>0
    d_x = ceil(m/grid_length)-1;
    d_y = ceil(n/grid_length);
else
    d_x = ceil(m/grid_length)-1;
    d_y = ceil(n/grid_length)-1;
end
goal_fix = [start(1)+d_x,start(2)+d_y];
end

function isobstacle=ISOBSTACLE(m,obstacle)
isobstacle=false;
for i=1:length(obstacle(:,1))
    if isequal(m(1:2),obstacle(i,1:2))
        isobstacle=true;
        return;
    end
end
end


function next=MOTIONMODEL(map)
next=map.grid_length*[-1,1,1.4;...
    0,1,1;...
    1,1,1.4;...
    1,0,1;...
    1,-1,1.4;...
    0,-1,1;...
    -1,-1,1.4;...
    -1,0,1];
end

function [typeflag,inc]=FINDTYPE(m,open,close)
typeflag=0;
inc=0;
%假如open为空
if isempty(open)
    typeflag=2;
end

%检查是否在close中(close不为空）
for i=1:length(close(:,1))
    if isequal( m(1:2) , close(i,1:2) )
        typeflag=1;
        return;
    end
end

%检查是否在open中
for j=1:length(open(:,1))
    if isequal( m(1:2) , open(j,1:2) )
        typeflag=3;%在open中
        inc=j;
        return;
    else
        typeflag=2;%不在open中
    end    
end
end


function h=h(m,goal)
h=abs(m(1)-goal(1))+abs(m(2)-goal(2));%曼哈顿距离
end

function  [inopenflag,inline]=ISINOPEN(array,open)
inopenflag=false;
inline=0;

if isempty(open)
    inopenflag=false;
end

 for i=1:length(open(:,1))
     if isequal(array(1:2),open(i,1:2))
         inopenflag=true;
         inline=i;
         return;
     end
 end
end