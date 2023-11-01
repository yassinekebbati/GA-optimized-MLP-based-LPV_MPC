%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/10/2021
% Control GA-LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out =RunGA(problem, params)

    % Problem
    CostFunction = problem.CostFunction;
    nVar = problem.nVar;
    VarSize = [1, nVar];
    VarMin = problem.VarMin;
    VarMax = problem.VarMax;
    
    
    % Parameters
    MaxIt = params.MaxIt;
    nPop = params.nPop;
    beta = params.beta;
    pC = params.pC;
    nC = round(pC*nPop/2)*2;
    gamma = params.gamma;
    mu = params.mu;
    sigma = params.sigma;
    
    
    %Template for empty individuals
    empty_individual.Position = [];  %empty_individual.Position = out.pop.Position; 
    empty_individual.Cost =[];

    
    % Best Solution Ever Found
    bestsol.Cost = inf;
    
    
    
    % Initialization
    pop = repmat(empty_individual, nPop, 1);
    for i = 1:nPop
        
        % Generate Random Solution
        pop(i).Position = unifrnd(VarMin, VarMax,VarSize);
        
        % Evaluate Solution
        pop(i).Cost = CostFunction(pop(i).Position);
        
        % Compare Solution to Best Solution Ever Found
        if pop(i).Cost < bestsol.Cost
            bestsol = pop(i);
        end
    end
    
    % Best Cost of Iterations
    bestcost = nan(MaxIt,1);
    
    
    % Main Loop
    for it = 1:MaxIt
        
        % Selection Probabilities
        c = [pop.Cost];
        avgc = mean(c);
        if avgc ~= 0
            c = c/avgc;
        end    
        probs = exp(-beta*c);
        
        % Initialize Offsprings Population
        popc = repmat(empty_individual, nC/2, 2);
        
        % Crossover
        for k = 1:nC/2
            
%             % Select Parents randomly
%             q = randperm(nPop);
%             p1 = pop(q(1));
%             p2 = pop(q(2));



%             if mod(k,2) == 0
%             % Select Parents 
                s1 = pop(RouetteWheelSelection(probs));
                s2 = pop(RouetteWheelSelection(probs));
            
      
%                 MatingPool = TournamentSelection([pop.Cost],nPop);
%                 MatingPool = TournamentSelection([pop.Cost],nPop);
                s3 = pop(TournamentSelection([pop.Cost],nPop));
                s4 = pop(TournamentSelection([pop.Cost],nPop));
%             end
                if s1.Cost < s2.Cost && s3.Cost < s4.Cost
                    p1 = s1;
                    p2 = s3;
                elseif s1.Cost < s2.Cost
                    p1 = s1;
                    p2 = s4;
                else
                    p1 = s2;
                    p2 = s3;
                end
                

            
            % Perform Crossover
            [popc(k,1).Position, popc(k,2).Position] = ...
                UniformCrossover(p1.Position, p2.Position, gamma);
%                 UniformCrossover(p1.Position, p2.Position);
%                 DoublePointCrossover(p1.Position, p2.Position);
%                 singlePointCrossover(p1.Position, p2.Position);
                
            
            
        end
        
        % Convert popc to Single-coloumn Matrix
        popc = popc(:);
        
        % Mutation
        for l = 1:nC
            
            popc(l).Position = Mutate(popc(l).Position, mu, sigma);
            
            %Check for variable bounds
            popc(l).Position = max(popc(l).Position, VarMin);
            popc(l).Position = min(popc(l).Position, VarMax);
            
            %Evaluation 
            popc(l).Cost = CostFunction(popc(l).Position);
            
            % Compare Solution to Best Solution Ever Found
            if popc(l).Cost < bestsol.Cost
                bestsol = popc(l);
            end
        
        end
        
       
        
        % Merge and Sort Populations
        pop = SortPopulation([pop; popc]);
        
        % Remove Extra Individuals 
        pop = pop(1:nPop);

        
        % Update Best Cost of Iteration
        bestcost(it) = bestsol.Cost;
        
        % Display Iteration Information
        disp(['Iteration' num2str(it) ': Best Cost =' num2str(bestcost(it))])
        
    end
    
    
    
    
    
    % Results
    out.pop = pop;
    out.bestsol = bestsol
    out.bestcost = bestcost;

end