function quadmodel
global A
% this function simulate the dynamics of the quadrotor, it will take the
% Euler angels and forces as inputs and give the linear accelerations WRT
% E-Frame and rotational accelerations WRT B-Frame.

A.X_ddot = (sin(A.psi)*sin(A.phi) + cos(A.psi)*sin(A.theta)*cos(A.phi))*A.U1/A.m;
A.Y_ddot = (-cos(A.psi)*sin(A.phi) + sin(A.psi)*sin(A.theta)*cos(A.phi))*A.U1/A.m;
A.Z_ddot = -A.g + (cos(A.theta)*cos(A.phi))*A.U1/A.m;

A.p_dot = ((A.Iyy - A.Izz)*A.q*A.r -A.Jtp*A.q*A.O + A.U2)/A.Ixx;
A.q_dot = ((A.Izz - A.Ixx)*A.p*A.r +A.Jtp*A.p*A.O + A.U3)/A.Iyy;
A.r_dot = ((A.Ixx - A.Iyy)*A.p*A.q + A.U4)/A.Izz;

A.phi_dot   = A.p + sin(A.phi)*tan(A.theta)*A.q + cos(A.phi)*tan(A.theta)*A.r;
A.theta_dot = cos(A.phi)*A.q - sin(A.phi)*A.r;
A.psi_dot   = sin(A.phi)/cos(A.theta)*A.q + cos(A.phi)/cos(A.theta)*A.r;

% Air friction model
A.X_ddot = A.X_ddot - A.X_dot*1.2;
A.Y_ddot = A.Y_ddot - A.Y_dot*1.2;

% Disturbance model
% A.X_ddot = A.X_ddot + A.X_dis/A.m;
% 
% A.Z_dis =0;
% A.X_dis =2*sin(A.Ts*0.5*A.counter) + 2*cos(A.Ts*0.5*A.counter) ;
% A.Y_dis =0;

% 
A.Z_dis = 0;
A.X_dis = 2;
A.Y_dis = 2;
% 
if (A.counter > 2000 && A.counter<2500)
    A.Z_dis = 0;
    A.X_dis = 2+sin(A.Ts*A.counter);
    A.Y_dis = 2+sin(A.Ts*A.counter);
end

if (A.counter > 3500 && A.counter<4500)
    A.Z_dis = 0;
    A.X_dis = 2+(0.5+cos(A.Ts*A.counter));
    A.Y_dis = 2+(0.5+cos(A.Ts*A.counter));
end

if A.counter > 4000
    A.Z_dis =0;
    A.X_dis =2;
    A.Y_dis =2;
end

A.X_ddot = A.X_ddot + A.X_dis;

A.Y_ddot = A.Y_ddot + A.Y_dis;

A.Z_ddot = A.Z_ddot + A.Z_dis;

A.psi_dot = A.psi_dot + A.psi_dis/A.Izz*A.Ts;

A.theta_dot = A.theta_dot + A.theta_dis/A.Iyy*A.Ts;

A.phi_dot = A.phi_dot + A.phi_dis/A.Ixx*A.Ts;


% Calculating the Z velocity & position
A.Z_dot = A.Z_ddot*A.Ts + A.Z_dot;
A.Z = A.Z_dot*A.Ts + A.Z;

% Calculating the X velocity & position
A.X_dot = A.X_ddot*A.Ts + A.X_dot;
A.X = A.X_dot*A.Ts + A.X;

% Calculating the Y velocity & position
A.Y_dot = A.Y_ddot*A.Ts + A.Y_dot;
A.Y = A.Y_dot*A.Ts + A.Y;

% Calculating p,q,r
A.p = A.p_dot*A.Ts+A.p;
A.q = A.q_dot*A.Ts+A.q;
A.r = A.r_dot*A.Ts+A.r;

% Calculating phi,theta,psi
A.phi = A.phi_dot*A.Ts+A.phi;
A.theta = A.theta_dot*A.Ts+A.theta;
A.psi = A.psi_dot*A.Ts+A.psi;

% % Plotting variables
A.Z_plot(A.counter) = A.Z;
A.Z_dot_plot(A.counter) = A.Z_dot;
A.Z_ref_plot(A.counter) = A.Z_des;
A.Z_plot_dis(A.counter) = A.Z_dis;
% 
A.X_plot(A.counter) = A.X;
A.X_dot_plot(A.counter) = A.X_dot;
A.X_ref_plot(A.counter) = A.X_des_EF;
A.X_plot_dis(A.counter) = A.X_dis;

A.Y_plot(A.counter) = A.Y;
A.Y_dot_plot(A.counter) = A.Y_dot;
A.Y_ref_plot(A.counter) = A.Y_des_EF;
A.Y_plot_dis(A.counter) = A.Y_dis;
% 
A.phi_plot(A.counter) = A.phi;
A.phi_ref_plot(A.counter) = A.phi_des;
% A.phi_dis_plot(A.counter) = A.phi_dis;
% 
A.theta_plot(A.counter) = A.theta;
A.theta_ref_plot(A.counter) = A.theta_des;
% A.theta_dis_plot(A.counter) = A.theta_dis;
% 
% A.psi_plot(A.counter) = A.psi;
% A.psi_ref_plot(A.counter) = A.psi_des;
% A.psi_dis_plot(A.counter) = A.psi_dis;


A.counter = A.counter + 1;

% Disturbance removal
if(A.X_dis ~= 0)
    A.X_dis = 0;
end

if(A.X_dis ~= 0)
    A.X_dis = 0;
end

if(A.Z_dis ~= 0)
    A.Z_dis = 0;
end

if(A.psi_dis ~= 0)
    A.psi_dis = 0;
end

if(A.theta_dis ~= 0)
    A.theta_dis = 0;
end

if(A.phi_dis ~= 0)
    A.phi_dis = 0;
end

end