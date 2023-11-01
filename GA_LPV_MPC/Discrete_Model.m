%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/10/2021
% Control LPV-MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ A, B, Cf,Cr ] = Discrete_Model( delta, vx, vy, curv, ey, theta_e, Ts, Cf, Cr)

    lf = 1.2;
    lr = 1.6;
    m  = 1575;
    I  = 2875;    
    g  = 9.81;
    mu = 0.05;
    rho = 1.225;
    CdA = 0.464; %1.6*0.29;
    
%     
%     Cf = 19000;
%     Cr = 33000;
%     net = network1
%     V = network1([vx;vy;acc;delta;theta_dot],xmean, xstdev, ymean,ystdev,);
%     V = pred([vx;vy;acc;delta;theta_dot] , xmean, xstdev, ymean,ystdev, mod);
%     Cf = abs(V(1));
%     Cr = abs(V(2));

    B = [ -Cf*sin(delta)/m      1 ;     %[delta accel]
          Cf*cos(delta)/m       0 ; 
          lf*Cf*cos(delta)/I    0 ;
            0                   0 ;
            0                   0];     

    B = Ts * B;

    A11 = -(mu*g)/vx - (0.5*rho*CdA*vx)/m;   % -mu; 
    A12 = (sin(delta) * Cf) / (m*vx);
    A13 = (sin(delta) * Cf * lf) / (m*vx) + vy;
    A22 = -(Cr + Cf * cos(delta)) / (m*vx);
    A23 = -(lf * Cf * cos(delta) - lr * Cr) / (m*vx) - vx;
    A32 = -(lf * Cf * cos(delta) - lr * Cr) / (I*vx);
    A33 = -(lf * lf * Cf * cos(delta) + lr * lr * Cr) / (I*vx);
    A45 = vx;
    A41 = sin(theta_e);
    A42 = cos(theta_e);
    A51 = -( curv * cos(theta_e)) / ( 1 - ey * curv );
    %A51 = -curv  / ( 1 - ey * curv );
    A52 = ( curv * sin(theta_e)) / ( 1 - ey * curv ); 

    A = [ A11  A12  A13  0  0 ;  %[vx]
           0   A22  A23  0  0 ;  %[vy]
           0   A32  A33  0  0 ;  %[wz] 
          A41  A42   0   0  0 ;  %[ey] 
          A51  A52   1   0  0 ];  %[theta_e]
          % 0   A42   0   0  A45;
          %A51  A52   1   0  0 ];  %[theta_e]
             
    A = eye(5) + Ts * A;

end

            
            