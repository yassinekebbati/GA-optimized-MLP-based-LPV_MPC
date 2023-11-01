%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/10/2021
% Control GA-LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pop = SortPopulation(pop)

    [~, so] = sort([pop.Cost]);
    pop = pop(so);
    
end