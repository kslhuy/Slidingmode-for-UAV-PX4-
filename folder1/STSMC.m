global A

% Sliding surface 
e = A.
ux = 
A.Y_des = A.Y_des_EF*cos(A.psi_kalman)-A.X_des_EF*sin(A.psi_kalman);    % calculating the desired X in BF
A.Y_BF = A.Y_kalman*cos(A.psi_kalman)-A.X_kalman*sin(A.psi_kalman);            % calculating X in BF
A.Y_dot_BF = A.Y_dot*cos(A.psi_kalman)-A.X_dot*sin(A.psi_kalman);    % calculating X_dot in BF

% PD controller for X_position
% A.phi_des = -1*(A.Y_KP*(A.Y_des - A.Y_BF) + A.Y_KD*A.Y_dot);

A.phi_des = asin(sin(A.psi_kalman)*ux - cos(A.psi_kalman)*uy)
A.theta_des = asin((cos(A.psi_kalman)*ux + sin(A.psi_kalman)*uy)/cos(A.phi_des))



if(abs(A.phi_des) > pi/3 )        % limiter
    A.phi_des = sign(A.phi_des)*pi/4;
end
if(abs(A.theta_des) > pi/3)        % limiter
    A.theta_des = sign(A.theta_des)*pi/;
end
