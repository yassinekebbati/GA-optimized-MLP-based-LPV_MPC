%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/10/2021
% Control GA-LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
close all;



%% Load Neural Network
load test_net

%% load References
load DLC_data   

% % Load postion data
load DLC_pos_data

% load wind disturbance data
load DLCwind_data

%% Problem Definition

problem.CostFunction = @(x) LPV_MPC(x,xmean, xstdev, ymean,ystdev, mod, Ts, UU, XX, XP, YP, THETAP,v_wind);
problem.nVar = 7;
problem.VarMin = [0.05 0.001 0.001 0.01 0.001 0.005 0.002];
problem.VarMax = [0.1 0.05 0.05 1.5 0.05 0.2 0.15];



%% GA Parameters

params.MaxIt = 3;     %maximum number of generations
params.nPop = 3;
params.beta = 0.8;        %Selection pressure 
params.pC = 0.8;        %percentage of children \parents
params.gamma = 0.2;     %parameter for extending possibilities for children beyond parents
params.mu = 0.3;       %mutation rate
params.sigma = 0.15;     %variance for mutation



%% Run GA 
out =RunGA(problem, params);

%%Extract solution 
disp 'solution :', out.bestsol.Position




%% Results
figure;
semilogy(out.bestcost, 'Linewidth' , 2); hold on;
xlabel('Iteration');
ylabel('Best Cost');
grid on;




