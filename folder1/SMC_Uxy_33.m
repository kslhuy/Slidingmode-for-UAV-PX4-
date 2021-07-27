function SMC_Uxy_33
      global A

      persistent SW1_x;
      persistent SW1_y;

      persistent SW2_x;
      persistent SW2_y;

      persistent SW3_x;
      persistent SW3_y;

      persistent xp_x;
      persistent xp_y;
      
      persistent ex;
      persistent ey;
      
      persistent errork1_y;
      persistent errork2_y;
      persistent vhatk1_y;
      persistent vhatk2_y; 
      
      persistent errork1_x;
      persistent errork2_x;
      persistent vhatk1_x;
      persistent vhatk2_x; 
      
%       persistent SW1;
%       persistent SW2;
%       persistent SW3;
%       persistent errork1;
%       persistent errork2;
%       persistent vhatk1;
%       persistent vhatk2; 
 
  % initialize persistent variables at beginning of simulation
  if A.init==0 
      errork1_x = 0;
      errork2_x = 0;
      vhatk1_x = 0;
      vhatk2_x = 0;
      
      
      errork1_y = 0;
      errork2_y = 0;
      vhatk1_y = 0;
      vhatk2_y = 0;
      
      SW3_x = 0;
      SW3_y = 0;
      
      SW1_x = 0;
      SW1_y = 0 ;
      
      SW2_x = 0;
      SW2_y = 0;
      
%       SW1=0;
%       SW2=0;
%       SW3=0;
      
%       errork1 = 0;
%       errork2 = 0;
%       vhatk1 = 0;
%       vhatk2 = 0;
%       
      
      A.Ux =0; 
      A.Uy =0;
      
      xp_x = [0 ; 0 ; 0];
      xp_y = [0 ; 0 ; 0];
      
      ex = [0 ; 0 ; 0];
      ey = [0 ; 0 ; 0];
   
  end

  X_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1_x + (2/(2*A.tau+A.Ts))*(A.X_des_EF - errork1_x);
  X_des_dot = min(3,abs(X_des_dot))*sign(X_des_dot);
  
  
  X_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2_x + (2/(2*A.tau+A.Ts))*(X_des_dot - errork2_x);
  X_des_ddot = min(6,abs(X_des_ddot))*sign(X_des_ddot);
  
  Y_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1_y + (2/(2*A.tau+A.Ts))*(A.Y_des_EF - errork1_y);
  Y_des_dot = min(3,abs(Y_des_dot))*sign(Y_des_dot);
  
  Y_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2_y + (2/(2*A.tau+A.Ts))*(Y_des_dot - errork2_y);
  Y_des_ddot = min(6,abs(Y_des_ddot))*sign(Y_des_ddot);
  
  % dirty derivative of pe to get vhat (error Z )


  k1x = 2;
  k2x = 3;
  k3x = 1.5;
  k4x = 1.5;
  
  k1y = 2;
  k2y = 3;
  k3y = 1.5;
  k4y = 1.5;
  
  cx = 2;
  cy = 2;
  
%   lamda1x =6.75;
  lamda2x = 25 ;
  lamda3x = 7;
%   lamda4x = 1.5;
  
%   lamda1y = 6.75;
  lamda2y = 25;
  lamda3y = 7;
%   lamda4y = 1.5;
  
  
  x1 = A.X_kalman;
  x1_des = A.X_des_EF;
  x1_dot_des = X_des_dot;
  
  x2_des = x1_dot_des;
  x2_dot_des = X_des_ddot;
  
  y1 = A.Y_kalman;
  y1_des = A.Y_des_EF;
  y1_dot_des = Y_des_dot;
  
  y2_des = y1_dot_des;
  y2_dot_des = Y_des_ddot;
  
  % Observator 
       u_obs_x = A.Ux ;
       u_obs_y = A.Uy ;
       
%      
       A.Ux_comp(A.counter) = u_obs_x ;  
       A.Uy_comp(A.counter) = u_obs_y ;  

  % PArameter for calulation 
%        Phi = [1 A.Ts A.Ts^2/2;0 1 A.Ts;0 0 1];
%        c = 10;
  % Calcule differentiator    

       [x_0, x_1,x_2] = differentiator3(x1,xp_x,A.Ts,u_obs_x,6,lamda2x,lamda3x);
       [y_0, y_1,y_2] = differentiator3(y1,xp_y,A.Ts,u_obs_y,6,lamda2y,lamda3y);
       
       %update         
       xp_x = [x_0; x_1;x_2];
       xp_y = [y_0; y_1;y_2];
       
%        ex = [ex1;ex2;ex3];
%        ey = [ey1;ey2;ey3];
       
        x1_hat =  x_0;
        x2_hat =x_1;
         
        y1_hat =  y_0;
        y2_hat = y_1;

        dis_x = x_2;
        dis_y = y_2;
        
        
        %plot data 
        A.x_es(A.counter) = x_0;
        A.x_dot_es(A.counter) = x_1;
        A.dis_x_es(A.counter) = dis_x;
        
        A.y_es(A.counter) = y_0;
        A.y_dot_es(A.counter) = y_1;
        A.dis_y_es(A.counter) = dis_y;
        
        %error
       
       
        
        
        dis_x_filtre = lowp(dis_x,0.5,0.01);
        dis_y_filtre = lowp(dis_y,0.5,0.01);
        
        
        
        A.dis_x_es_fil(A.counter) = dis_x_filtre;
  
        A.dis_y_es_fil(A.counter) = dis_y_filtre;
  
  error_x = x1 - x1_des;
  error_dot_x = x2_hat - x2_des;
  
  
  error_y = y1 - y1_des;
  error_dot_y = y2_hat - y2_des;
  
   %Sliding sur face 
  s_x = cx*error_x + error_dot_x ;
  s_y = cy*error_y + error_dot_y ;
  
  SW1_x = SW1_x+ sat(x1 - x1_hat)*A.Ts;
  
  SW2_x = SW2_x+ sat(s_x)*A.Ts;
  
  SW3_x = SW3_x+ s_x*A.Ts;
  
  SW1_y = SW1_y+ sat(y1 - y1_hat)*A.Ts;
  
  SW2_y = SW2_y+ sat(s_y)*A.Ts;
  
  SW3_y = SW3_y+ s_y*A.Ts;
% %   
%   A.Ux = ( +dis_x_filtre -cx*x2_hat + cx*x1_dot_des - lamda3x*SW1_x - lamda2x*(abs(x1-x1_hat)^1/3) *sat(x1 - x1_hat) + x2_dot_des - k1x*(abs(s_x)^1/2) *sat(s_x) - k2x*s_x - k3x*SW2_x - k4x*SW3_x);
%   A.Uy = ( +dis_y_filtre -cy*y2_hat + cy*y1_dot_des - lamda3y*SW1_y - lamda2y*(abs(y1-y1_hat)^1/3) *sat(y1 - y1_hat) + y2_dot_des - k1y*(abs(s_y)^1/2) *sat(s_y) - k2y*s_y - k3y*SW2_y - k4y*SW3_y);
  A.Ux = (  -cx*x2_hat + cx*x1_dot_des - lamda3x*SW1_x - lamda2x*(abs(x1-x1_hat)^1/3) *sat(x1 - x1_hat) + x2_dot_des - k1x*(abs(s_x)^1/2) *sat(s_x) - k2x*s_x - k3x*SW2_x - k4x*SW3_x);
  A.Uy = (  -cy*y2_hat + cy*y1_dot_des - lamda3y*SW1_y - lamda2y*(abs(y1-y1_hat)^1/3) *sat(y1 - y1_hat) + y2_dot_des - k1y*(abs(s_y)^1/2) *sat(s_y) - k2y*s_y - k3y*SW2_y - k4y*SW3_y);
      
  A.Uxx(A.counter) = A.Ux;
  A.Uyy(A.counter) = A.Uy;

      
    
      
% Roll pitch dezire 
% % 
      A.theta_des = atan2(A.Ux*cos(A.psi_kalman) + A.Uy*sin(A.psi_kalman),A.Uz );
      A.phi_des = atan2(cos(A.theta_kalman)*(A.Ux*sin(A.psi_kalman) - A.Uy*cos(A.psi_kalman)),A.Uz);

      
%      
%       A.phi_des = asin((A.m*(A.Ux*sin(A.psi_kalman) - A.Uy*cos(A.psi_kalman) - sin(A.psi_kalman)*dis_x + cos(A.psi_kalman)*dis_y))/A.U1);
%      A.theta_des = asin((A.m*(A.Ux*cos(A.psi_kalman) - A.Uy*sin(A.psi_kalman) - cos(A.psi_kalman)*dis_x - sin(A.psi_kalman)*dis_y))/(A.U1*cos(A.phi_des)));

  
%% 
  if(abs(A.theta_des) > pi/6)        % limiter
    A.theta_des = sat(A.theta_des)*pi/6;
  end
  if(abs(A.phi_des) > pi/6)        % limiter
    A.phi_des = sat(A.phi_des)*pi/6;
  end
  
  % update stored variables
  errork1_x = A.X_des_EF;
  errork2_x = X_des_dot;
  vhatk1_x = X_des_dot;
  vhatk2_x = X_des_ddot;
  
  errork1_y = A.Y_des_EF;
  errork2_y = Y_des_dot;
  vhatk1_y = Y_des_dot;
  vhatk2_y = Y_des_ddot;

end
