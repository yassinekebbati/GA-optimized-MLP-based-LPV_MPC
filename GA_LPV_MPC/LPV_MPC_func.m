%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/10/2021
% Control LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ controller ] = LPV_MPC_func( Hp, SchedVars_Limits, nx, nu, q1, q2, q3, q4, q5, r1, r2 )
% function [ controller ] = LPV_MPC_fnc( Hp, SchedVars_Limits, nx, nu )
x       = sdpvar(nx,1,Hp+1,'full');    % [vx, vy, w, ey, etheta]
u       = sdpvar(nu,1,Hp,'full');      % [delta, acceleration]
du      = sdpvar(nu,1,Hp,'full');
pastu   = sdpvar(nu,1,'full');
ref     = sdpvar(nx,1,Hp+1,'full');
eps     = sdpvar(nx+nu,1,Hp+1,'full');


A       = sdpvar(nx,nx,Hp,'full'); 
B       = sdpvar(nx,nu,Hp,'full');

StateSets = sdpvar(nx*2,nx+1,Hp,'full'); 
InputSets = sdpvar(nu*2,nu+1,Hp,'full');
%WSet      = sdpvar(nx*2,nx+1,'full');

Qvx     = 1/SchedVars_Limits(1,2);
Qvy     = 1/SchedVars_Limits(2,2);
Qw      = 1/SchedVars_Limits(3,2);
Qdelta  = 1/SchedVars_Limits(4,2);
Qaccel  = 1/SchedVars_Limits(5,2);
Qey     = 1/SchedVars_Limits(6,2);
Qetheta = 1/SchedVars_Limits(7,2);


% Q   = 0.9*diag([ 0.02*Qvx   0.01*Qvy   0.01*Qw  0.75*Qey  0.01*Qetheta ]);     %0.9*diag([ 0.5*Qvx   0.0001*Qvy   0.001*Qw  0.5*Qey  0.0001*Qetheta ]); % [vx vy w ey etheta]
% R   = 0.1*diag([ 0.5*Qdelta 0.2*Qaccel ]);    %0.1*diag([ 0.7*Qdelta 0.3*Qaccel ]);                             % [delta accel]


% Q   = diag([q4*Qey  q5*Qetheta ]);
% R   = diag([ r1*Qdelta 0 ]); 
Q   = diag([ q1*Qvx   q2*Qvy   q3*Qw  q4*Qey  q5*Qetheta ]);     
R   = diag([ r1*Qdelta r2*Qaccel ]);    




% a = 0.7; 
% rho = 5;
% epsi = 0.5;

%%
objective = 0;

for k = 1:Hp
    if k==1
        constraints = [du(:,1,k) == u(:,1,k) - pastu;
                       %WSet(:,1:end-1) * (x_hat - x(:,1)) <= WSet(:,end) 
                       ];
    else
        constraints = [constraints; du(:,1,k) == u(:,1,k) - u(:,1,k-1)];
    end   
    
    objective = objective   + ( ref(:,k)-x(:,k) )' * Q * ( ref(:,1,k)-x(:,1,k) ) ...
                        + du(:,1,k)' * R * du(:,1,k);
                    
%     objective = objective   + ( ref(4:5,k)-x(4:5,k) )' * Q * ( ref(4:5,1,k)-x(4:5,1,k) ) ...
%                         + du(:,1,k)' * R * du(:,1,k);
    
%     objective = objective   + a^(-k)*( ref(:,k)-x(:,k) )' * Q * ( ref(:,1,k)-x(:,1,k) ) ...
%                             + du(:,1,k)' * R * du(:,1,k); %+ rho*epsi^2;
                        
    constraints = [constraints;

        x(:,k+1) == A(:,:,k) * x(:,k) + B(:,:,k) * u(:,k);

%         Ax * x(:,k) <= bx ; % New State Sets obtained by means of the reachable set R
%         
%         Au * u(:,k) <= bu ; % New Input Sets obtained by means of the reachable set R        
        
        StateSets(:,1:end-1,k) * x(:,k) <= StateSets(:,end,k) ; % New State Sets obtained by means of the reachable set R
        
        InputSets(:,1:end-1,k) * u(:,k) <= InputSets(:,end,k) ; % New Input Sets obtained by means of the reachable set R

       ]; 

end


% objective = objective + ( ref(:,k+1)-x(:,k+1) )' * Q * ( ref(:,k+1)-x(:,k+1) );

constraints = [ constraints;
                StateSets(:,1:end-1,k) * x(:,k+1) <= StateSets(:,end,k) ;
              ]; 
    
parameters_in = {x(:,1), pastu, ref, A, B, StateSets, InputSets};

solutions_out = {u, x(:,1,2:end), objective};



%% Quadprog:
options = sdpsettings('solver','quadprog');%'gurobi' 

controller = optimizer( constraints, objective, options, parameters_in, solutions_out);

end
