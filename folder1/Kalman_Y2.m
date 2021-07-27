function Kalman_Y2
global A
persistent Pp Q R xp A1 H I xp_dot
if isempty(Pp)
    Pp=1;     Q=5;    R=100;     
    xp=0;     A1=0.5;      H=1;     I=eye(1);   xp_dot = 0; 
end 
% Compute priori
    xp_dot = ((-cos(A.psi_kalman)*sin(A.phi_kalman) + sin(A.psi_kalman)*sin(A.theta_kalman)*cos(A.phi_kalman))*A.U1/A.m - A.Y_dot*1.2)*A.Ts + xp_dot;
    xp=xp_dot*A.Ts + xp;
    Pp=A1*Pp*A1'+Q;
% Measurement update 
    K = Pp*H'*inv(H*Pp*H'+R);
    A.Y_kalman = xp+K*(A.Y_meas-H*xp);

    A.Y_kalman_plot(A.counter) = A.Y_kalman;
% Update covariance  
    Pp=(I-K*H)*Pp;  
    xp = A.Y_kalman;
% End or function