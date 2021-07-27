function SMC_attitude
global A

  persistent SW_theta;
  persistent errork1_theta;
  persistent errork2_theta;
  persistent vhatk1_theta;
  persistent vhatk2_theta; 
  
  persistent SW_phi;
  persistent errork1_phi;
  persistent errork2_phi;
  persistent vhatk1_phi;
  persistent vhatk2_phi; 
  
  persistent SW_psi;
  persistent errork1_psi;
  persistent errork2_psi;
  persistent vhatk1_psi;
  persistent vhatk2_psi; 
  % initialize persistent variables at beginning of simulation
  if A.init==0 
      SW_theta = 0;
      errork1_theta = 0;
      errork2_theta = 0;
      vhatk1_theta = 0;
      vhatk2_theta = 0;
      
      SW_phi = 0;
      errork1_phi = 0;
      errork2_phi = 0;
      vhatk1_phi = 0;
      vhatk2_phi = 0;
      
      SW_psi = 0;
      errork1_psi = 0;
      errork2_psi = 0;
      vhatk1_psi = 0;
      vhatk2_psi = 0;
      
  end

  
  % dirty derivative of pe to get vhat (error Z )

  
  theta_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1_theta + (2/(2*A.tau+A.Ts))*(A.theta_des - errork1_theta);
  theta_des_dot = min(3,abs(theta_des_dot))*sign(theta_des_dot);
  
  
  theta_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2_theta + (2/(2*A.tau+A.Ts))*(theta_des_dot - errork2_theta);
  theta_des_ddot = min(5,abs(theta_des_ddot))*sign(theta_des_ddot);
  
  error_theta = A.theta_des - A.theta_kalman;
  error_dot_theta = theta_des_dot - A.theta_dot;
  
  phi_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1_phi + (2/(2*A.tau+A.Ts))*(A.phi_des - errork1_phi);
  phi_des_dot = min(3,abs(phi_des_dot))*sign(phi_des_dot);
  
  phi_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2_phi + (2/(2*A.tau+A.Ts))*(phi_des_dot - errork2_phi);
  phi_des_ddot = min(5,abs(phi_des_ddot))*sign(phi_des_ddot);
  
  error_phi = A.phi_des - A.phi_kalman;
  error_dot_phi = phi_des_dot - A.phi_dot;
  
  psi_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1_psi + (2/(2*A.tau+A.Ts))*(A.psi_des - errork1_psi);
  psi_des_dot = min(3,abs(psi_des_dot))*sign(psi_des_dot);
  
  psi_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2_psi + (2/(2*A.tau+A.Ts))*(psi_des_dot - errork2_psi);
  psi_des_ddot = min(5,abs(psi_des_ddot))*sign(psi_des_ddot);
  
  error_psi = A.psi_des - A.psi_kalman;
  error_dot_psi = psi_des_dot - A.psi_dot;

%% 
  % paper Kali
%   
    %Sliding sur face 
      rho_phi = 5;
      K1_phi = 5;
      K2_phi = 4;
      
      rho_theta = 5;
      K1_theta = 5;
      K2_theta = 4;

      rho_psi = 5;
      K1_psi = 5;
      K2_psi = 2;

      s_phi = error_dot_phi + rho_phi*error_phi;
      s_theta = error_dot_theta + rho_theta*error_theta;
      s_psi = error_dot_psi + rho_psi*error_psi;
      
      sW_theta = SW_theta + sat(s_theta)*A.Ts/2;
      sW_phi = SW_phi + sat(s_phi)*A.Ts/2;
      sW_psi = SW_psi + sat(s_psi)*A.Ts/2;



    %   Nonlinear decoupled
%     U2 = phi , U3 = theta , U4 = yaw



      A.U2 = A.Ixx *( phi_des_ddot + rho_phi*error_dot_phi - psi_des_dot*theta_des_dot*(A.Iyy-A.Izz)/A.Ixx + K1_phi*sqrt(abs(s_phi))*sat(s_phi) + K2_phi*sW_phi);
      A.U3 = A.Iyy *( theta_des_ddot + rho_theta*error_dot_theta - psi_des_dot*phi_des_dot*(A.Izz-A.Ixx)/A.Iyy + K1_theta*sqrt(abs(s_theta))*sat(s_theta) + K2_theta*sW_theta);
      A.U4 = A.Izz *( psi_des_ddot + rho_psi*error_dot_psi - phi_des_dot*theta_des_dot*(A.Ixx-A.Iyy)/A.Izz + K1_psi*sqrt(abs(s_psi))*sat(s_psi) + K2_psi*sW_psi);
%Conclure  GOODDDDDDD !!!!!!!!!!!!
% 



  
  % update stored variables
  errork1_psi = A.psi_des;
  errork2_psi = psi_des_dot;
  vhatk1_psi = psi_des_dot;
  vhatk2_psi = psi_des_ddot;
  
  errork1_phi = A.phi_des;
  errork2_phi = phi_des_dot;
  vhatk1_phi = phi_des_dot;
  vhatk2_phi = phi_des_ddot;
  
  errork1_theta = A.theta_des;
  errork2_theta = theta_des_dot;
  vhatk1_theta = theta_des_dot;
  vhatk2_theta = theta_des_ddot;
  
  SW_phi = sW_phi;
  SW_theta = sW_theta;
  SW_psi = sW_psi;
end
