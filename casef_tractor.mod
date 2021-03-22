# ==============================================================================
# AMPL Model File for "Trajectory Planning for a Tractor with Multiple Trailers
# in Extremely Narrow Environments: A Unified Approach".  (License BSD)

# ==============================================================================

#   Copyright (C) 2019 Bai Li
#   Users must cite the following article if they utilize these codes:
#   Bai Li et al., "Trajectory Planning for a Tractor with Multiple Trailers in
#   Extremely Narrow Environments: A Unified Approach", Accepted in ICRA 2019.

# ==============================================================================

# If there are inquiries, please contact libai@zju.edu.cn
#
# (Very Important) The AMPL utilized in this pack is just a TRIAL version.
# The users MUST delete AMPL.exe in this current version after trying it, and
# then apply for their own license files through following the official instructions at
# https://ampl.com/try-ampl/request-a-full-trial/

# ==============================================================================

param NE == 80; 		# Number of finite elements in OCDT
param NC; 			# Number of tractor + trailers ##
param NO; 				# Number of obstacles

var tf >= 1; 			# Completion time, t ranges between 0 and tf.
var hi = tf / NE; 		# Time interval of each finite element
set I := {1..NE};		
set V := {1..NC};
set O := {1..NO};

# Location of each vertex of each obstacle

param PPP{i in O, j in {1..4}, k in {1..2}}; 	# Vertex of each obstacle
param CenterPP{i in O, k in {1..2}}; 			# Geometric center of each obstacle
param AREA{i in O}; 							# Area of each obstacle

param initial_config {i in {1..(7+NC-1)}}; 		# Initial configuration
param terminal_config {i in {1..5}}; 			# Terminal configuration

########### Bounds on the state/control profiles ###################### 
param amax == 0.25;
param vmax == 2.0;
param wmax == 0.5;
param phymax == 0.7;

########### Geometric size of the tractor-trailer vehicle #############
param TN == 0.25;
param TL == 1.5;
param TM == 0.25;
param TB == 1;

param L2 == 3;

param TT1 == 1;
param TT2 == 1;

########### Declaration of the variabes ##############################
var x{i in I, k in V};
var y{i in I, k in V};
var theta{i in I, k in V};
var v{i in I};
var a{i in I};
var phy{i in I};
var w{i in I};

var AX{i in I, k in V};
var BX{i in I, k in V};
var CX{i in I, k in V};
var DX{i in I, k in V};

var AY{i in I, k in V};
var BY{i in I, k in V};
var CY{i in I, k in V};
var DY{i in I, k in V};

############# Minimization objective ###############
minimize objective_:
1;


########### For the sake of discretization accuracy
s.t. time_constraint:
tf = 1;

############# DAEs #################### 

###### ODEs for the towing tractor ####

s.t. DIFF_dx1dt {i in {2..NE}}:
x[i,1] - x[i-1,1] = hi * v[i-1] * cos(theta[i-1,1]);

s.t. DIFF_dy1dt {i in {2..NE}}:
y[i,1] - y[i-1,1] = hi * v[i-1] * sin(theta[i-1,1]);

s.t. DIFF_dvdt {i in {2..NE}}:
v[i] - v[i-1] = hi * a[i-1];

s.t. DIFF_dthetadt {i in {2..NE}}:
theta[i,1] - theta[i-1,1] = hi * (tan(phy[i-1])) * v[i-1] / TL;

s.t. DIFF_dphydt {i in {2..NE}}:
phy[i] - phy[i-1] = hi * w[i-1];


########## ODEs for the trailers ###########################



########## AEs for the whole systems ###########################


s.t. RELATIONSHIP_AX1 {i in I}:
AX[i,1] = x[i,1] + (TL + TN) * cos(theta[i,1]) - TB * sin(theta[i,1]);

s.t. RELATIONSHIP_BX1 {i in I}:
BX[i,1] = x[i,1] + (TL + TN) * cos(theta[i,1]) + TB * sin(theta[i,1]);

s.t. RELATIONSHIP_CX1 {i in I}:
CX[i,1] = x[i,1] - TM * cos(theta[i,1]) + TB * sin(theta[i,1]);

s.t. RELATIONSHIP_DX1 {i in I}:
DX[i,1] = x[i,1] - TM * cos(theta[i,1]) - TB * sin(theta[i,1]);

s.t. RELATIONSHIP_AY1 {i in I}:
AY[i,1] = y[i,1] + (TL + TN) * sin(theta[i,1]) + TB * cos(theta[i,1]);

s.t. RELATIONSHIP_BY1 {i in I}:
BY[i,1] = y[i,1] + (TL + TN) * sin(theta[i,1]) - TB * cos(theta[i,1]);

s.t. RELATIONSHIP_CY1 {i in I}:
CY[i,1] = y[i,1] - TM * sin(theta[i,1]) - TB * cos(theta[i,1]);

s.t. RELATIONSHIP_DY1 {i in I}:
DY[i,1] = y[i,1] - TM * sin(theta[i,1]) + TB * cos(theta[i,1]);




############# Two-point boundary conditions #################### 
##
s.t. EQ_starting_x :
x[1,1] = initial_config[1];

s.t. EQ_starting_y :
y[1,1] = initial_config[2];

s.t. EQ_starting_theta {pp in {1..NC}}:
theta[1,pp] = initial_config[pp+2];

#s.t. EQ_starting_theta1 :
#theta[1,1] = initial_config[3];

#s.t. EQ_starting_theta2 :
#theta[1,2] = initial_config[4];

#s.t. EQ_starting_theta3 :
#theta[1,3] = initial_config[5];

#s.t. EQ_starting_theta4 :
#theta[1,4] = initial_config[6];

s.t. EQ_starting_phy :
phy[1] = initial_config[NC+3];

s.t. EQ_starting_v :
v[1] = initial_config[NC+4];

s.t. EQ_starting_a :
a[1] = initial_config[NC+5];

s.t. EQ_starting_w :
w[1] = initial_config[NC+6];
##


s.t. EQ_ending_v :
v[NE] = terminal_config[3];

s.t. EQ_ending_a :
a[NE] = terminal_config[4];

s.t. EQ_ending_w :
w[NE] = terminal_config[5];

############## Bounded constraints #################### 

s.t. Bonds_v {i in I}:
v[i]^2 <= vmax^2;

data;
param NO:= include Number_obstacle;
param: PPP := include Current_vertex;
param: CenterPP := include Center;
param AREA := include Area;
param initial_config := include Initial_config;
param terminal_config :=  include Terminal_config;
param NC := include NC_config;