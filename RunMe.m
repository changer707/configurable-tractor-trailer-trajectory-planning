% ==============================================================================
% MATLAB Source Codes for "Trajectory Planning for a Tractor with Multiple
% Trailers in Extremely Narrow Environments: A Unified Approach".

% ==============================================================================

%   Copyright (C) 2019 Bai Li
%   User must cite the following article if they utilize these codes for
%   new publications: Bai Li et al., "Trajectory Planning for a Tractor
%   with Multiple Trailers in Extremely Narrow Environments: A Unified
%   Approach", To appear in IEEE 2019 International Conference on Robotics
%   and Automation (ICRA).

% ==============================================================================

% If there are inquiries, feel free to contact libai@zju.edu.cn
%
% (Very Important) The AMPL utilized in this pack is just a TRIAL version.
% The users must delete AMPL.exe in this version after trying it, and
% then apply for their own valid license files through following the
% official instructions at https://ampl.com/try-ampl/request-a-full-trial/

% ==============================================================================
clear
clc
close all

global polygon_obstacle_vertex
%%%%%%%%  Experiment setup %%%%%%%%

% Case 1


%% Terminal configuration %%
% We create a box with the geometric center being (x_center_tf, y_center_tf)
% The box is 16 m length and 2.6 m width
x_center_tf = 8;        % geometric center along x axis 
y_center_tf = 0;         % geometric center along y axis
v_tf = 0;                    % v(t = tf)
a_tf = 0;                    % a(t = tf)
w_tf = 0;                   % w(t = tf)
box_l = 16;
box_w = 2.6;
box_vertex = zeros(8,1);% 右上-左上（顺时针）
box_vertex(1)=x_center_tf+0.5*box_l;box_vertex(2)=y_center_tf+0.5*box_w;
box_vertex(3)=x_center_tf+0.5*box_l;box_vertex(4)=y_center_tf-0.5*box_w;
box_vertex(5)=x_center_tf-0.5*box_l;box_vertex(6)=y_center_tf-0.5*box_w;
box_vertex(7)=x_center_tf-0.5*box_l;box_vertex(8)=y_center_tf+0.5*box_w;
%% Obstacle setup %%
% Vertex points of N_obs obstable are record in a 1 x (8*N_obs) vector. For example,
% suppose we have one rectangular obstacle with vertexes V1 (v1x, v2x), V2
% (v2x, v2y), V3(v3x, v3y), and V4(v4x, v4y), then the vector should be
% [v1x, v1y, v2x, v2y, v3x, v3y, v4x, v4y].
% 默认障碍物为四边形

%  polygon_obstacle_vertex = [-30 -15 20 -15 20 -25 -30 -25 -30 5 20 5 20 10 -30 10 0 -12.5 10 -12.5 10 -7.5 0 -7.5];%lane change
 polygon_obstacle_vertex = [0 -10 20 -10 20 -2.5 0 -2.5 0 2.5 20 2.5 20 10 0 10];%park
%% Initial configuration %%
% lane change:(-20,-10)    park:(-15,-20)
x0_1 = -10;         % x_1(t = 0)
y0_1 = -15;         % y_1(t = 0)
phy_0 = 0;          % phy(t = 0)
v_0 = 0;              % v(t = 0)
a_0 = 0;              % a(t = 0)
w_0 = 0;             % w(t = 0)
%% 可配置参数
NC = 2;            % number of tractor+trailer
flag_offaxle = 1;  % offaxle-1 ， onaxle-0
%% boundary_constraints
% start-end points
% boundary_constraints = [x0_1, y0_1, theta0_1, theta0_2, theta0_3, theta0_4, phy_0, v_0, a_0, w_0, x_center_tf, y_center_tf, v_tf, a_tf, w_tf];
boundary_constraints = zeros(1,NC+11);
for i =1:NC
    boundary_constraints(i+2) = pi/2; %park
end
boundary_constraints(1)= x0_1;boundary_constraints(2)=y0_1;boundary_constraints(2+NC+1)=phy_0;boundary_constraints(2+NC+2)=v_0;
boundary_constraints(2+NC+3)=a_0;boundary_constraints(2+NC+4)=w_0;boundary_constraints(2+NC+5)=x_center_tf;boundary_constraints(2+NC+6)=y_center_tf;
boundary_constraints(2+NC+7)=v_tf;boundary_constraints(2+NC+8)=a_tf;boundary_constraints(2+NC+9)=w_tf;
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
