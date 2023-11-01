%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/10/2021
% Control LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


yalmip('clear')
clear
close all
clc

%% load cost function weights
load Q3;  %Q4


q1 = Q(1);
q2 = Q(2);
q3 = Q(3); 
q4 = Q(4); 
q5 = Q(5); 
r1 = Q(6); 
r2 = Q(7);




%% Load Neural Network

% load test_net
load best_net


%% load References

% load DLC_data    %double lane change track data
load test_data   %test track data


% % Load postion data
% load DLC_pos_data
load Pos_data


% load DLCwind_data
load wind_data



ini_index   = 1; 
Vx      = XX(1,ini_index:end);
Vy      = XX(2,ini_index:end);
Wz      = XX(3,ini_index:end);        
Ey      = XX(4,ini_index:end);
ThetaE  = XX(5,ini_index:end);
steer   = UU(1,ini_index:end);
accel   = UU(2,ini_index:end);


Vx_ref      = Vx;
Vy_ref      = Vy;
Omega_ref   = Wz;
Ey_ref      = Ey;
ThetaE_ref  = ThetaE;
Steer_ref   = steer;
Accel_ref   = accel;

Curv_ref = Wz./Vx;

steer       = Steer_ref(1); 
accel       = Accel_ref(1); 
vx          = Vx_ref(1);
vy          = Vy_ref(1); 
omega       = Omega_ref(1);

x0          = [vx; vy; omega; 0; 0];  %MPC state vector 

vehicle_states  = [vx; vy; omega; 0; 0; 0]; %Simulator state vector

xp          = 0;
yp          = 0;
yaw         = 0;

tXlast      = XP(1);
tYlast   	= YP(1);
tThetalast 	= THETAP(1);
s(1)      	= 0;


%% LPV-MPC-reference design:
nu = 2;     % number of system inputs
nx = 5;     % number of system states
Hp = 9;     % prediction horizon

% Limits for the scheduling variables:
SchedVars_Limits = [  0.1 25;    % vx
                     -3 3;       % vy
                     -2.5 2.5;   % wz
                     -0.67 0.67; % delta
                     -2 5;       % acceleration
                     -0.25 0.25; % lateral error
                     -4 5];      % orientation error

% Controller object definition:       
% LPV_MPC_controller = LPV_MPC_func( Hp, SchedVars_Limits, nx, nu );
LPV_MPC_controller = LPV_MPC_func( Hp, SchedVars_Limits, nx, nu, q1, q2, q3, q4, q5, r1, r2 );  


A_vec = zeros(nx,nx,Hp);
B_vec = zeros(nx,nu,Hp);

% State constraint matrix:
StateSets = zeros(nx*2,nx+1,Hp);
AxSet = [eye(5); -eye(5)];
bxSet = [SchedVars_Limits(1,2);
         SchedVars_Limits(2,2);
         SchedVars_Limits(3,2);
         SchedVars_Limits(6,2);
         SchedVars_Limits(7,2);
        -SchedVars_Limits(1,1);
        -SchedVars_Limits(2,1);
        -SchedVars_Limits(3,1);
        -SchedVars_Limits(6,1);
        -SchedVars_Limits(7,1)];
for i=1:Hp
    StateSets(:,:,i) = [AxSet, bxSet];
end

% Input constraint matrix:
InputSets = zeros(nu*2,nu+1,Hp);
AuSet = [eye(2); -eye(2)];
buSet = [SchedVars_Limits(4,2);
     SchedVars_Limits(5,2);
     -SchedVars_Limits(4,1);
     -SchedVars_Limits(5,1)];
for i=1:Hp
    InputSets(:,:,i) = [AuSet, buSet];
end



counter = 0;
for i = 1:2537   %252 %use this iteration nbr for double lane change
    i
    if i == 1
        cf(i) = 19000;
        cr(i) = 33000;
    end
    V = pred([XX(4,1); XX(5,1);UU(2,1);UU(1,1);Wz(1)] , xmean, xstdev, ymean,ystdev, mod);  %use this when using : best_net/test_net
    cf(i+1) = abs(V(1));
    cr(i+1) = abs(V(2));


    % Reference vector:
    for j=i:i+Hp
        REF(:,1,j-i+1) = [Vx_ref(j) 0 0 0 0 ]';
    end
    
    % LPV system instantiation:
    if i == 1      
        for j=1:Hp
            curvature(j-i+1) = 0;
        end


        for j=1:Hp 
            [ A_vec(:,:,j), B_vec(:,:,j), cf(j), cr(j)  ] = Discrete_Model ...
                ( Steer_ref(j), Vx_ref(j), Vy_ref(j), curvature(j), Ey_ref(j), ThetaE_ref(j), Ts ,  cf(i+1), cr(i+1));

        end
        
        PASTU = [Steer_ref(1) Accel_ref(1)]'; 
    else       

        

        for j=1:Hp
            curvature(j) = Curv_ref(j);  
            [ A_vec(:,:,j), B_vec(:,:,j), cf(i+1+j-1), cr(i+1+j-1)] = Discrete_Model ...
                ( UU(1,j), XX(1,j), XX(2,j), curvature(j), XX(4,j), XX(5,j), Ts, cf(i+1), cr(i+1));    

        end    
 
    end
   
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Optimization problem:
    inputs = {x0, PASTU, REF, A_vec, B_vec, StateSets, InputSets};
    tic
    [solutions,diagnostics] = LPV_MPC_controller{inputs};
    ET_MPC(i) = toc;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if diagnostics == -1
        UU = PASTU;
    elseif diagnostics == 0
        UU   = solutions{1}; 
        XX   = solutions{2};
        CF(i)= solutions{3};
    else         
        error('Nan result.')
    end
    
    PASTU = UU(:,1,1); % Storing current optimal solution
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %\\ Vehicle simulation:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    T = Ts*i:Ts/30:Ts*i+Ts; 
    steer       = UU(1,1,1);
    accel       = UU(2,1,1);
    
    
   
    V_wind = v_wind(i)/3;
    Road_slope = 0;    %include road slop with this
    
    
    [T,x] = ode45(@(t,x) nonlinear_model(t,x, [steer; accel; Curv_ref(i); V_wind; Road_slope]), T, vehicle_states);  
    
    vehicle_states = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)]; % [vx vy w ey s etheta]
    
    x0 = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,6)]; % [vx vy w ey etheta]
    
    % Middle-road computation:
    tXref(1)    = tXlast;
    tYref(1)    = tYlast;
    tThetaref(1)= tThetalast;
    for j = 1:length(XX(1,:))
        s(i+j) = s(i+j-1) + ( ( XX(1,j)*cos(XX(5,j)) - XX(2,j)*sin(XX(5,j)) ) / (1-XX(4,j)*curvature(j)) ) * Ts;
        tXref(j+1)= XP(i);
        tYref(j+1)= YP(i);
        tThetaref(j+1)= THETAP(i);
    end
    
    tXlast = tXref(2);
    tYlast = tYref(2);
    tThetalast = tThetaref(2);    
        
    % Error-based pose computation:
    for k=1:length(XX(1,:))
        yaw(k) = tThetaref(k) + x(k,6);             
        xp(k) = tXref(k) - x(k,4)*sin(yaw(k));      
        yp(k) = tYref(k) + x(k,4)*cos(yaw(k));      
    end   

    %% Storing variables for plotting:
    CA(:,i)     = UU(:,1,1);
    STATES_MPC(:,i) = XX(:,1,1);
    STATES_VEHICLE(:,i) = x0;
    POSE(:,i)   = [xp(1) yp(1) yaw(1)]';
    
    figure(1), hold on, plot(POSE(1,:), POSE(2,:),'r'), grid on, plot(XP(1:i), YP(1:i),'g--'), grid on; %
    
end

figure(2)
subplot(7,1,1), plot(CA(1,1:i),'r'), ylabel('steering'), grid on , hold on              %, plot(Steer_ref(1:i),'--b')
subplot(7,1,2); plot(CA(2,1:i),'r'), ylabel('acceleration'), grid on , hold on          %, plot(Accel_ref(1:i),'--b')
subplot(7,1,3), plot(STATES_VEHICLE(1,1:i)), hold on, ylabel('vx'), plot(Vx_ref(1:i),'--r'), ylabel('vx'), grid on
subplot(7,1,4); plot(STATES_VEHICLE(2,1:i)), ylabel('vy'), grid on, hold on             %, plot(Vy_ref(1:i),'--r')
subplot(7,1,5), plot(STATES_VEHICLE(3,1:i)), ylabel('w [deg/s]'), grid on, hold on      %, plot(Omega_ref(1:i),'--r')
subplot(7,1,6); plot(STATES_VEHICLE(4,1:i)), ylabel('ey [m]'), grid on; hold on,        %plot(Ey_ref(1:i),'--r')
subplot(7,1,7); plot(STATES_VEHICLE(5,1:i)), ylabel('thetae [rad]'), grid on; hold on,  % plot(ThetaE_ref(1:i),'--r')

% figure(3), plot(cf,'r--'), grid on;
% figure(4), plot(cr,'k--'), grid on;

drawnow

mse1 = sqrt(sum(STATES_VEHICLE(4,1:i).^2)/252);
mse2 = sqrt(sum(STATES_VEHICLE(5,1:i).^2)/252);
mse3 = sqrt(sum((STATES_VEHICLE(1,1:i)-Vx_ref(1:i)).^2)/252);
rmse1 = sqrt(mse1);
rmse2 = sqrt(mse2);
rmse3 = sqrt(mse3);
rmse = rmse1 + rmse2 + rmse3
MPC_time = mean(ET_MPC)




