%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/10/2021
% Control GA-LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [y1 , y2] = UniformCrossover(x1, x2, gamma)

%     alpha = rand(size(x1));
    alpha = unifrnd(-gamma, 1+gamma, size(x1));
    
    y1 = alpha.*x1 + (1 - alpha).*x2;
    y2 = alpha.*x2 + (1 - alpha).*x1;

end