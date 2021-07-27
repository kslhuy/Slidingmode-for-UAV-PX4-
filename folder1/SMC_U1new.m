function SMC_U1new

global A

  persistent SW1;
  persistent SW2;
  persistent SW3;
  persistent errork1;
  persistent errork2;
  persistent vhatk1;
  persistent vhatk2; 
  persistent xp; 
  
  % initialize persistent variables at beginning of simulation
  if A.init==0 
      errork1 = 0;
      errork2 = 0;
      vhatk1 = 0;
      vhatk2 = 0;
      SW1 = 0;
      SW2 = 0;
      SW3 = 0;
      xp = [0;0;0];
  end
 
  
  cz = 1;
  lamda3z = 20;
  lamda2z = 5;
  k1z = 1;
  k2z = 1;
  k3z = 1;
  k4z = 1;
  % dirty derivative of pe to get vhat (error Z )
  
  Z_des_EF_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1 + (2/(2*A.tau+A.Ts))*(A.Z_des - errork1);
  Z_des_EF_dot = min(1,abs(Z_des_EF_dot))*sign(Z_des_EF_dot);
  
  Z_des_EF_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2 + (2/(2*A.tau+A.Ts))*(Z_des_EF_dot - errork2);
  Z_des_EF_ddot = min(2,abs(Z_des_EF_ddot))*sign(Z_des_EF_ddot);
  
    
  z1 = A.Z_kalman;
  z1_des = A.Z_des;
  z1_dot_des = Z_des_EF_dot;
  
  z2_des = z1_dot_des;
  z2_dot_des = Z_des_EF_ddot;
  
  % Observator 
       u_obs_z = ((cos(A.phi_kalman)*cos(A.theta_kalman))/A.m)* A.U1 - A.g ;
       
%      
       A.Uz_comp(A.counter) = u_obs_z ;  

  % PArameter for calulation 
       Phi = [1 A.Ts A.Ts^2/2;0 1 A.Ts;0 0 1];
       c = 5;
  % Calcule differentiator    

       [x_0, x_1,x_2] = differentiator2(z1,xp,A.Ts,u_obs_z,Phi,c);
        
       xp = [x_0; x_1;x_2];
        z1_hat =  x_0;
        z2_hat = x_1;
        
        A.z_es(A.counter) = x_0;
        A.z_dot_es(A.counter) = x_1;
        A.dis_z_es(A.counter) = x_2;
  
  error = z1 - z1_des;
  error_dot = z2_hat - z2_des;
   %Sliding sur face 
  s = cz*error + error_dot ;
  
  SW1 = SW1+ sat(z1 - z1_hat)*A.Ts;
  
  SW2 = SW2+ sat(s)*A.Ts;
  
  SW3 = SW3+ s*A.Ts;
  
  A.Uz = (A.g + z2_dot_des -cz*z2_hat + cz*z1_dot_des - lamda3z*SW1 - lamda2z*(abs(z1-z1_hat)^1/3) *sat(z1 - z1_hat) - k1z*(abs(s)^1/2) *sat(s) - k2z*s - k3z*SW2 - k4z*SW3);
  A.U1 = A.m/(cos(A.phi_kalman)*cos(A.theta_kalman))*A.Uz;
  A.Uzz(A.counter) = A.Uz;
  

  % update stored variables
  errork1 = A.Z_des;
  errork2 = Z_des_EF_dot;
  vhatk1 = Z_des_EF_dot;
  vhatk2 = Z_des_EF_ddot;
 
  
end