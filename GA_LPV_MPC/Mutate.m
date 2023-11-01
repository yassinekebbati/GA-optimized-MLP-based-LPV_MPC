%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/10/2021
% Control GA-LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function y = Mutate(x, mu, sigma)
    
    flag = (rand(size(x)) < mu);
    
    y = x;
    r = randn(size(x));
    y(flag) = x(flag) + sigma*r(flag);


end