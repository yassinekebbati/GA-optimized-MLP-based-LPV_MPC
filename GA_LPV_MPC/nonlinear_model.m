%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/10/2021
% Control LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = nonlinear_model(t, states, u)
 
    lf = 1.2;
    lr = 1.6;
    m  = 1575;
    I  = 2875;
    Cf = 19000;
    Cr = 33000;
    g  = 9.81;
    
    % States
    vx      = states(1);
    vy      = states(2);
    w       = states(3);
    ey      = states(4);      
    s       = states(5);     
    epsi    = states(6);      

    % Inputs:
    delta   = u(1);
    a       = u(2); 
    curv    = u(3);  
    V_wind  = u(4);
    Road_slope = u(5);

    if abs(vx) > 0.1
        % Front and rear slip angles:
        a_F = atan((vy + lf*w)/vx) - delta;
        a_R = atan((vy - lr*w)/vx);
    else
        a_F = 0.0;
        a_R = 0.0;    
    end
    
    % Pacejka tire model:
%     FyF = -pacejka_tire_model(a_F);
%     FyR = -pacejka_tire_model(a_R);
    FyF = -Cf*a_F;
    FyR = -Cr*a_R;
%    
%     out1 = mfeval('MagicFormula61_Parameters.tir', [15450 0 a_F 0 0 vx], 111);
%     out2 = mfeval('MagicFormula61_Parameters.tir', [15450 0 a_R 0 0 vx], 111);
%     FyF= out1(:,2);
%     FyR= out2(:,2);
 
%     if abs(a_F) > 30.0/180.0*pi || abs(a_R) > 30.0/180.0*pi
%         disp("WARNING: Large slip angles in simulation")
%         a_F*180/pi
%         a_R*180/pi
%     end
    
    F_drag = 0.05*vx;
    Fy_wind = 0.5 * 1.225 * 1.5 * V_wind^2;
    
    dx(1,1) =   a - F_drag  -  FyF*sin(delta)/m  +  w*vy - g*sin(Road_slope) ;  %[Vx]    
    dx(2,1) = ( FyF*cos(delta) + FyR - Fy_wind ) / m  -  w*vx ;                 %[Vy]
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr - Fy_wind*(lf-lr) ) / I  ;           %[Wz]
    dx(4,1) = vx*sin(epsi)+vy*cos(epsi);                                        %[ey] 
    dx(5,1) = (vx*cos(epsi)-vy*sin(epsi)) / (1-ey*curv);                        %[s]
    dx(6,1) = w - ( (vx*cos(epsi)-vy*sin(epsi))*curv / (1-ey*curv) );           %[etheta = epsi]
    
    
end

