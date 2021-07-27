function SMC_U1

global A

  persistent SW;
  persistent errork1;
  persistent errork2;
  persistent vhatk1;
  persistent vhatk2; 
  % initialize persistent variables at beginning of simulation
  if A.init==0 
      errork1 = 0;
      errork2 = 0;
      vhatk1 = 0;
      vhatk2 = 0;
      SW = 0;
  end
  rho_z = 1;
  K1_z = 2;
  K2_z = 2;
  % dirty derivative of pe to get vhat (error Z )
  
  Z_des_EF_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1 + (2/(2*A.tau+A.Ts))*(A.Z_des - errork1);
  Z_des_EF_dot = min(0.5,abs(Z_des_EF_dot))*sign(Z_des_EF_dot);
  Z_des_EF_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2 + (2/(2*A.tau+A.Ts))*(Z_des_EF_dot - errork2);
  Z_des_EF_ddot = min(1,abs(Z_des_EF_ddot))*sign(Z_des_EF_ddot);
  
  error =  A.Z_des-A.Z_kalman ;
  error_dot = Z_des_EF_dot - A.Z_dot  ;
   %Sliding sur face 
  s = rho_z*error + error_dot ;
     
  SW = SW+ K2_z*sat(s/0.8)*A.Ts;
%   
  A.Uz = (A.g + Z_des_EF_ddot + rho_z*error_dot + K1_z*sqrt(abs(s))*sat(s/0.8) + SW);
  A.U1 = A.m/(cos(A.phi_kalman)*cos(A.theta_kalman))*A.Uz;
  A.Uzz(A.counter) = A.Uz;
  
%   F = up + ui + ud;
% 
%   A.U1 = A.m*(A.g + F)/cos(A.phi_kalman)/cos(A.theta_kalman);
%   A.U1_plot(A.counter) = A.U1;
%   
  % update stored variables
  errork1 = A.Z_des;
  errork2 = Z_des_EF_dot;
  vhatk1 = Z_des_EF_dot;
  vhatk2 = Z_des_EF_ddot;
 
  
end