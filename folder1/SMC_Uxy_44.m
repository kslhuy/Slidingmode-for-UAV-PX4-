function SMC_Uxy_44
      global A

      persistent SW1_x;
      persistent SW1_y;

      persistent SW2_x;
      persistent SW2_y;

   

      persistent xp_x;
      persistent xp_y;
      
      persistent errork1_y;
      persistent errork2_y;
      persistent vhatk1_y;
      persistent vhatk2_y; 
      
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
      
      
      errork1_y = 0;
      errork2_y = 0;
      vhatk1_y = 0;
      vhatk2_y = 0;
      

      
      SW1_x = 0;
      SW1_y = 0 ;
      
      SW2_x = 0;
      SW2_y = 0;

      A.Ux =0; 
      A.Uy =0;
      
      xp_x = [0 ; 0 ; 0];
      xp_y = [0 ; 0 ; 0];
   
  end

  X_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1 + (2/(2*A.tau+A.Ts))*(A.X_des_EF - errork1);
  X_des_dot = min(2,abs(X_des_dot))*sign(X_des_dot);
  
  A.plot_X_des_dot(A.counter) = X_des_dot;
  
  X_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2 + (2/(2*A.tau+A.Ts))*(X_des_dot - errork2);
  X_des_ddot = min(3,abs(X_des_ddot))*sign(X_des_ddot);
  
  Y_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1_y + (2/(2*A.tau+A.Ts))*(A.Y_des_EF - errork1_y);
  Y_des_dot = min(2,abs(Y_des_dot))*sign(Y_des_dot);
  
  A.plot_Y_des_dot(A.counter) = Y_des_dot;
  
  Y_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2_y + (2/(2*A.tau+A.Ts))*(Y_des_dot - errork2_y);
  Y_des_ddot = min(3,abs(Y_des_ddot))*sign(Y_des_ddot);
  
  % dirty derivative of pe to get vhat (error Z )

    % Control gain
    k1x = 2;
    k2x =2;

    k1y = 2;
    k2y = 2;

    cx = 1.5;
    cy = 1.5;

    %Observer gain
    lamda1x = 5.5;
    lamda2x = 21;
    lamda3x = 10;

    lamda1y = 5.5;
    lamda2y = 21;
    lamda3y = 10;

  
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
    u_obs_x = A.Ux  ;
    u_obs_y = A.Uy  ;

    
    A.Ux_comp(A.counter) = u_obs_x ;  
    A.Uy_comp(A.counter) = u_obs_y ;  


  % Calcule differentiator    

    [x_0, x_1,x_2,e1] = differentiator3(x1,xp_x,A.Ts,u_obs_x,lamda1x,lamda2x,lamda3x);
    [y_0, y_1,y_2,e2] = differentiator3(y1,xp_y,A.Ts,u_obs_y,lamda1y,lamda2y,lamda3y);

    %update         
    xp_x = [x_0; x_1;x_2];
    xp_y = [y_0; y_1;y_2];

    x1_hat =  x_0;
    x2_hat = x_1;
%     x2_hat = lowp(x_1,5,0.01);
    y1_hat =  y_0;
    y2_hat = y_1;
%     y2_hat = lowp(y_1,5,0.01);

    %         dis_x = x_2;
    %         dis_y = y_2;
%     
    dis_x = min(8,abs(x_2))*sign(x_2);
    dis_y = min(8,abs(x_2))*sign(x_2);

    %plot data 
    A.x_es(A.counter) = x_0;
    A.x_dot_es(A.counter) = x_1;
    A.dis_x_es(A.counter) = dis_x;

    A.y_es(A.counter) = y_0;
    A.y_dot_es(A.counter) = y_1;
    A.dis_y_es(A.counter) = dis_y;
% 
     A.dis_x_es_fil(A.counter) = e1;

     A.dis_y_es_fil(A.counter) = e2;
    % error 
      error_x = x1 - x1_des;
      error_dot_x = x2_hat - x2_des;

      error_y = y1 - y1_des;
      error_dot_y = y2_hat - y2_des;

       %Sliding sur face 
      s_x = cx*error_x + error_dot_x ;
      s_y = cy*error_y + error_dot_y ;

      % Term intergal of error estimation
     
      % Term intergal of SWSMC
      SW2_y = SW2_y+ k2x*sat(s_y/0.8)*A.Ts;
      SW2_x = SW2_x+ k2y*sat(s_x/0.8)*A.Ts;

 
%       A.Ux = (  - cx*x2_hat + cx*x1_dot_des - lamda3x*SW1_x - lamda2x*(abs(x1-x1_hat)^(1/3))*sat(x1 - x1_hat) + x2_dot_des - k1x*(abs(s_x)^(1/2))*sat(s_x) - k2x*SW2_x);
%       A.Uy = (  - cy*y2_hat + cy*y1_dot_des - lamda3y*SW1_y - lamda2y*(abs(y1-y1_hat)^(1/3))*sat(y1 - y1_hat) + y2_dot_des - k1y*(abs(s_y)^(1/2))*sat(s_y) - k2y*SW2_y);
      if (A.counter < 50)
          A.Ux = (  - cx*x2_hat + cx*x1_dot_des  - lamda2x*(abs(x1-x1_hat)^(1/3))*sat((x1 - x1_hat)/0.8) + x2_dot_des - k1x*(abs(s_x)^(1/2))*sat(s_x/0.8) - SW2_x);
          A.Uy = (  - cy*y2_hat + cy*y1_dot_des  - lamda2y*(abs(y1-y1_hat)^(1/3))*sat((y1 - y1_hat)/0.8) + y2_dot_des - k1y*(abs(s_y)^(1/2))*sat(s_y/0.8) - SW2_y);

      else
          A.Ux = (  - cx*x2_hat + cx*x1_dot_des -dis_x  - lamda2x*(abs(x1-x1_hat)^(1/3))*sat((x1 - x1_hat)/0.01) + x2_dot_des - k1x*(abs(s_x)^(1/2))*sat(s_x/0.8) - SW2_x);
          A.Uy = (  - cy*y2_hat + cy*y1_dot_des  -dis_y - lamda2y*(abs(y1-y1_hat)^(1/3))*sat((y1 - y1_hat)/0.01) + y2_dot_des - k1y*(abs(s_y)^(1/2))*sat(s_y/0.8) - SW2_y);

      end
      A.Uxx(A.counter) = A.Ux;
      A.Uyy(A.counter) = A.Uy;

      
   
% Roll pitch dezire 
% % 
      A.theta_des = atan2(A.Ux*cos(A.psi_kalman) + A.Uy*sin(A.psi_kalman),A.Uz );
      A.phi_des = atan2(cos(A.theta_des)*(A.Ux*sin(A.psi_kalman) - A.Uy*cos(A.psi_kalman)),A.Uz);

      
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
  errork1 = A.X_des_EF;
  errork2 = X_des_dot;
  vhatk1 = X_des_dot;
  vhatk2 = X_des_ddot;
  
  errork1_y = A.Y_des_EF;
  errork2_y = Y_des_dot;
  vhatk1_y = Y_des_dot;
  vhatk2_y = Y_des_ddot;

end
