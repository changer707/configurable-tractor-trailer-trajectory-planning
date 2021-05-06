% 可配置tractor+trailer轨迹规划
% 2021.4.7 9:52
%功能：
%（1）NE可配置     （2）obj.fun 路径最短+避障    （3）终点航向角回正     （4）终点区域大小、方位可配置
%（5）车体大小与障碍物异步渐增（相同参数下，求解速度加快）
%效果：
% 4车倒车入库可实现
%可继续改进之处：
% （1）获取最初的初始解——可以避免无法解释的奇异行为
% 
% ****需要修改****
% 1.RunMe：可配置参数、场景构造
% 2.matlab--ampl：PrepareTrajectoryPlanning.m    SetTwoPointBoundaryConditionsToFiles.m  case.mod  
% 3.目标函数、约束、ipopt参数：case.mod  ipopt.opt   
% ==============================================================================
clear
clc
close all
diary 'C:\Users\17863\Desktop\Astar初始解\mylog.txt'
global polygon_obstacle_vertex  dynamic_constraints
%% 可配置参数
NE = 160;          % 离散时间步数
NC = 4;            % tractor+trailer 数目
flag_offaxle = 0;  % 连轴/离轴 offaxle-1 ， onaxle-0
                                                        % lane change:(-20,-10)    park:(-15,-20)
x0_1 =   -5  ;         % tractor起始x坐标 x_1(t = 0)     % 泊车：（-10，-15，pi/2）正向， (-5,15,pi/2)倒车 , 平行泊车：（-6,2，pi）
y0_1 =   15  ;         % tractor起始y坐标 y_1(t = 0)     % 右转：（-2.5，-15，pi/2）90度，  (5,-15,0.75*pi)45度
theta0_all = 0.5*pi  ; % tractor+trailer起始航向角 theta_all(t = 0)      % 调头：（5，-12.5，pi）
                            % 终点：(12.5,-2.5)
x_center_tf = 12.5;         % 矩形中心x坐标 12.5
y_center_tf = -2.5;         % 矩形中心y坐标 -2.5
theta_tf = pi;              % tractor+trailer终点航向 0  ,  泊车/平行泊车：pi

TN = 0.25; %tractor前悬
TL = 1.5; %tractor轴距
TM = 0.25; %tractor后悬
TT1 = 1; %trailer前悬
TT2 = 1; %trailer后悬
TB = 1; %tractor+trailer车宽
L2 = 3; %tractor-trailer相连长度
shape_param = [ TN TL TM TB L2 TT1 TT2 ];

phymax = 0.7;
amax = 0.25;
vmax = 2.0;
wmax = 0.5;
dynamic_constraints = [phymax,amax,vmax,wmax];
%% Terminal configuration %%
% We create a box with the geometric center being (x_center_tf, y_center_tf)
% The box is 16 m length and 2.6 m width
v_tf = 0;                    % v(t = tf)
a_tf = 0;                    % a(t = tf)
w_tf = 0;                   % w(t = tf)
alfa_tf = 0;              % 终点框的位置
             
box_l = 16;                 % default:16*2.6
box_w = 2.6;
box_vertex = GetBoxVertex(x_center_tf,y_center_tf,box_w,box_l,alfa_tf);  %得到终点矩形的顶点坐标
%% 场景构造 %%
% Vertex points of N_obs obstable are record in a 1 x (8*N_obs) vector. For example,
% suppose we have one rectangular obstacle with vertexes V1 (v1x, v2x), V2
% (v2x, v2y), V3(v3x, v3y), and V4(v4x, v4y), then the vector should be
% [v1x, v1y, v2x, v2y, v3x, v3y, v4x, v4y].
% 默认障碍物为四边形

% 90度右转
% obs1=[0 -5 20 -5 20 -10 0 -10];
% obs2=[0 10 10 10 10 5 0 5];
% obs3=[-20 -5 -10 -5 -10 -10 -20 -10];
% obs4=[-20 10 -10 10 -10 5 -20 5];

% 45度右转
% obs1=[0 -5 20 -5 20 -20 15 -20];
% obs1=[5 -5 20 -5 20 -15 5 -10];

% 倒车入库
obs1 = [5 -5 20 -5 20 -10 5 -10];
obs2 = [5 0 20 0 20 10 5 10];

% 调头
% obs1 = [0 -5 40 -5 40 -10 0 -10];

%平行泊车
% obs1 = [-5 -1 4 -1 4 -4 -5 -4];
% obs2 = [21 -1 28 -1 28 -4 21 -4];
% obs3 = [4 -4 21 -4 21 -5 4 -5];
 polygon_obstacle_vertex = [obs1,obs2];
%% Initial configuration %%
phy_0 = 0;          % phy(t = 0)
v_0 = 0;              % v(t = 0)
a_0 = 0;              % a(t = 0)
w_0 = 0;             % w(t = 0)
%% boundary_constraints
% start-end points
% boundary_constraints = [x0_1, y0_1, theta0_1, theta0_2, theta0_3, theta0_4, phy_0, v_0, a_0, w_0, x_center_tf, y_center_tf, v_tf, a_tf, w_tf];
boundary_constraints = zeros(1,NC+12);
for i =1:NC
    boundary_constraints(i+2) =   theta0_all; %park
end
boundary_constraints(1)= x0_1;boundary_constraints(2)=y0_1;boundary_constraints(2+NC+1)=phy_0;boundary_constraints(2+NC+2)=v_0;
boundary_constraints(2+NC+3)=a_0;boundary_constraints(2+NC+4)=w_0;boundary_constraints(2+NC+5)=x_center_tf;boundary_constraints(2+NC+6)=y_center_tf;
boundary_constraints(2+NC+7)=v_tf;boundary_constraints(2+NC+8)=a_tf;boundary_constraints(2+NC+9)=w_tf;
boundary_constraints(2+NC+10)= theta_tf;
%% Prepare for Trajectory planning %%

PrepareTrajectoryPlanning; % 把 boundary_constraints 拆分成初状态数组和末状态数组

testcurve;   % A*轨迹规划+曲线二次规划
%% Trajectory planning %%
AdaptivelyHomotopicWarmStartingApproach;


%% Solution illustration
if (is_success)
    figure (101)
    plot(store_gamma);
    xlabel('Number of Cycle');
    ylabel('\gamma_a_c_h_i_e_v_e_d')
    title('Evolution of \gamma_a_c_h_i_e_v_e_d');
    
    figure (102)
    plot(store_step);
    xlabel('Number of Cycle');
    ylabel('step')
    title('Evolution of step');
    
    figure (201)
    plot(store_gamma1);
    xlabel('Number of Cycle');
    ylabel('\gamma1_a_c_h_i_e_v_e_d')
    title('Evolution of \gamma1_a_c_h_i_e_v_e_d');
    
    figure (202)
    plot(store_step1);
    xlabel('Number of Cycle');
    ylabel('step1')
    title('Evolution of step1');

    figure (103)
    DrawTrajectories; % Plot the optimized trajectories.
    figure(104)
    draw_paths(NC,box_vertex,flag_offaxle);
%     DrawPath;
    %%% If reader has interest, the dynamic process can be observed through
    %%% the following function.
%     VideoGeneration;
end

load x.txt
load y.txt
sum1=0;sum2=0;
for i=1:2:length(x)-2
    sum1 = sum1+sqrt( (x(i)-x(i+2)).^2 + (y(i)-y(i+2)).^2 );
end
for i = 2:2:length(x)-2
    sum2 = sum2+sqrt( (x(i)-x(i+2)).^2 + (y(i)-y(i+2)).^2 );
end
sum = sum1+sum2;
diary off