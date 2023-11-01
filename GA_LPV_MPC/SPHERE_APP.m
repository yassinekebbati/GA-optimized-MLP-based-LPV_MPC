%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/10/2021
% Control GA-LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
% close all;




%% Problem Definition for sphere function

problem.CostFunction = @(x) Sphere(x);   
problem.nVar = 5;      
problem.VarMin = [-10 -10 -5 -1 5];
problem.VarMax = [10 10 5 1 8];


%% GA Parameters

params.MaxIt = 100;     %maximum number of generations
params.nPop = 10;       %Population size

params.beta = 0.8;        %Selection pressure 
params.pC = 0.8;        %percentage of children \parents
params.gamma = 0.2;     %parameter for extending possibilities for children beyond parents
params.mu = 0.25;       %mutation rate
params.sigma = 0.15;     %variance for mutation




%% Run GA 

out =RunGA(problem, params);

%% Results

figure;
% plot(out.bestcost, 'Linewidth' , 2);
semilogy(out.bestcost, 'Linewidth' , 2); hold on;
xlabel('Iteration');
ylabel('Best Cost');
grid on;




