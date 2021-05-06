% This script is used to do trajectory planning

%% Adaptively homotopic warm-starting approach (i.e. Algorithm 1 in the ICRA paper)
step = 0.2;                  % Initial setting of step
alpha = 0.5;                % Setting of \alpha_reduce
Nexpand = 10;            % Threshold of successive successful cycles that would make step increase 
epsilon_exit = 1e-5;     % Threshold to exit Algorithm 1 with a failure
epsilon_0 = 0.05;         % Gamma value related to Subproblem 0
epsilon_01 = 0.05;   %初始为点（小）
step1 = 0.2;
%% Begin with \gamma = epsilon_0;
% Write geometric centers of all the obstacles in a file
GenerateGeometricCenters;
GenerateVertexes(epsilon_0,shape_param);
GenerateBody(epsilon_01,shape_param);
figure(105)
%% Pre-solution with simple initialization   原始的初始解
% Note that this small design is not mentioned in detail in our ICRA paper.
if NC>1
    if ~flag_offaxle % on-axle
        WriteInitialGuessForNLP(NE,NC,Bazierpath,shape_param,theta0_all);
    !ampl rf.run
    !ampl r0.run
    !ampl r1.run
    
%% Formally solve subproblem 0
    !ampl r2.run
    else % off-axle
    !ampl rf_offaxle_1.run
    !ampl r0_offaxle_1.run
    !ampl r1_offaxle_1.run
    !ampl r2_offaxle_1.run
    end
else
    !ampl rf_tractor.run
    !ampl r0_tractor.run
    !ampl r1_tractor.run
    !ampl r2_tractor.run
end
load flag.txt;

if (flag == 0) % If the simplest sub-problem fails, exit immediately.
    error 'Solution Failure at Subproblem 0.';
end
    current_trajectory(NC,box_vertex);
%% Initialization of the parameters.
gamma_achieved = epsilon_0;
gamma1_ach = epsilon_01;
counter = 0;

store_gamma = [];
store_step = [];

%% Sequential NLP-solving process    
%% 障碍递增
% The following while loops never terminates until the originla problem
% (i.e. the one with \gamma_achieved == 1) is solved successfully OR
% epsilon_exit related threshold is activated.
i = 105;
while (gamma_achieved ~= 1)
    store_step = [store_step, step]; % Record the current step value
    if ((gamma_achieved + step) <= 1) % Check if the next gamma_trial would exceed the upper bound
        gamma_trial = gamma_achieved + step;
%         gamma1 = gamma1_ach + step;
    else % If it exceeds, it would be set to 1
        gamma_trial = 1;
%         gamma1 = 1;
    end
    GenerateVertexes(gamma_trial,shape_param);
%     GenerateBody(gamma1,shape_param);
    store_gamma = [store_gamma, gamma_achieved]; % Record the current successful gamma_achieved
    % 解优化问题
    if NC>1
        if ~flag_offaxle
        !ampl r2.run
        else
            !ampl r2_offaxle_1.run
        end
    else
        !ampl r2_tractor.run
    end
    load flag.txt; % Check if the current optimization trail succeeds: 1 = succeeds, 0 = fails.

    if (flag == 1) % If succeeds
        gamma_achieved = gamma_trial; % Update the current successful gamma_trail as gamma.
%         gamma1_ach = gamma1;
        counter = counter + 1; % Increase the counter by 1.
    else % If fails
        step = step .* alpha; % Reduce "step" because a failure happens
        counter = 0; % Reset the counter
    end

    if (counter >= Nexpand)
        step = step ./ alpha;
        counter = 0;
    end
    i = i+1;
    figure(i)
    current_trajectory(NC,box_vertex);
    fclose('all');

    if (step <= epsilon_exit)
        error Intermediate_Subproblem_Solution_Failure_due_to_epsilon_exit_Criterion
    end
end
%% 车体递增
store_gamma1 = [];
store_step1 = [];
counter = 0;
while(gamma1_ach~=1)
     store_step1 = [store_step1, step1]; % Record the current step value
    if ((gamma1_ach + step1) <= 1) % Check if the next gamma_trial would exceed the upper bound
%         gamma_trial = gamma_achieved + step;
        gamma1 = gamma1_ach + step1;
    else % If it exceeds, it would be set to 1
%         gamma_trial = 1;
        gamma1 = 1;
    end
    GenerateVertexes(1,shape_param);
    GenerateBody(gamma1,shape_param);
    store_gamma1 = [store_gamma1, gamma1_ach]; % Record the current successful gamma_achieved
    % 解优化问题
    if NC>1
        if ~flag_offaxle
        !ampl r2.run
        else
            !ampl r2_offaxle_1.run
        end
    else
        !ampl r2_tractor.run
    end
    load flag.txt; % Check if the current optimization trail succeeds: 1 = succeeds, 0 = fails.

    if (flag == 1) % If succeeds
%         gamma_achieved = gamma_trial; % Update the current successful gamma_trail as gamma.
        gamma1_ach = gamma1;
        counter = counter + 1; % Increase the counter by 1.
    else % If fails
        step1 = step1 .* alpha; % Reduce "step" because a failure happens
        counter = 0; % Reset the counter
    end

    if (counter >= Nexpand)
        step1 = step1 ./ alpha;
        counter = 0;
    end
    i = i+1;
    figure(i)
    current_trajectory(NC,box_vertex);
    fclose('all');

    if (step1 <= epsilon_exit)
        error Body_Intermediate_Subproblem_Solution_Failure_due_to_epsilon_exit_Criterion
    end
end

is_success = 1;