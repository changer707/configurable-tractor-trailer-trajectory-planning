% 可配置tractor+trailer轨迹规划
% 2021.4.7 9:52
%功能：
%（1）NE可配置     （2）obj.fun 路径最短+避障    （3）终点航向角回正     （4）终点区域大小、方位可配置     
%效果：
% 4车倒车入库可实现
%可继续改进之处：
% （1）为了保证求解成功率：先少车无障碍的场景——多车狭窄的场景（车数的渐增、障碍的渐大、障碍渐逼近终点）
% 
% ****需要修改****
% 1.RunMe：可配置参数、场景构造
% 2.matlab--ampl：PrepareTrajectoryPlanning.m    SetTwoPointBoundaryConditionsToFiles.m  case.mod  
% 3.目标函数、约束、ipopt参数：case.mod  ipopt.opt   
% ==============================================================================
clear
clc
close all

global polygon_obstacle_vertex
%% 可配置参数
NE = 160;          % 离散时间步数
NC = 4;            % tractor+trailer 数目
flag_offaxle = 0;  % 连轴/离轴 offaxle-1 ， onaxle-0
                                                        % lane change:(-20,-10)    park:(-15,-20)
x0_1 =   5  ;         % tractor起始x坐标 x_1(t = 0)     % 泊车：（-10，-15，pi/2）正向，   (-5,15,pi/2)倒车
y0_1 =   -15  ;         % tractor起始y坐标 y_1(t = 0)     % 右转：（-2.5，-15，pi/2）90度，  (5,-15,0.75*pi)45度
theta0_all = 0.75*pi  ; % tractor+trailer起始航向角 theta_all(t = 0)
                            % 终点：(12.5,-2.5)
x_center_tf = -5;         % 矩形中心x坐标 12.5
y_center_tf = -5;         % 矩形中心y坐标 -2.5
theta_tf = 0.75*pi;              % tractor+trailer终点航向 0
%% Terminal configuration %%
% We create a box with the geometric center being (x_center_tf, y_center_tf)
% The box is 16 m length and 2.6 m width
v_tf = 0;                    % v(t = tf)
a_tf = 0;                    % a(t = tf)
w_tf = 0;                   % w(t = tf)
alfa_tf = 0.75*pi;              % 终点框的位置
             
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
obs1=[0 -5 20 -5 20 -20 15 -20];
% obs1=[5 -5 20 -5 20 -15 5 -10];
% 倒车入库
% obs1 = [5 -5 20 -5 20 -10 5 -10];
% obs2 = [5 0 20 0 20 10 5 10];

 polygon_obstacle_vertex = [];%park
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
%% Other alternative cases %%
% % Case 2
% polygon_obstacle_vertex = [-12.81,2.42,-11.56,1.01,-15.19,-2.2,-16.44,-0.79,-5.85,5.12,-4.38,5.99,-2.05,2.08,-3.52,1.21,0.47,-2.18,0.38,-4.06,-4.47,-3.82,-4.38,-1.94, -10,5-8,-8,7-7.5,-5,-5-8,-10,-4-8];
% boundary_constraints = [-10,10,0,0,0,0,0,0,0,0,0,0,0,0,0];
%
% % Case 3
% polygon_obstacle_vertex = [0, 1.8, 10, 2, 10, 20, 0, 20, 10, 2, 10, 20, 20, 20, 20, 1, 0, -1.8, 20, -1.8, 20, -30, 0, -30];
% boundary_constraints = [-3,-10,pi/2,pi/2,pi/2,pi/2,0,0,0,0,8,0,0,0,0];

%% Prepare for Trajectory planning %%

PrepareTrajectoryPlanning; % 把 boundary_constraints 拆分成初状态数组和末状态数组


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
