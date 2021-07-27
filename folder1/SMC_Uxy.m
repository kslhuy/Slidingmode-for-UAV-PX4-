function SMC_Uxy
      global A

      persistent SW;
      persistent errork1;
      persistent errork2;
      persistent vhatk1;
      persistent vhatk2; 

      persistent SW_y;
      persistent errork1_y;
      persistent errork2_y;
      persistent vhatk1_y;
      persistent vhatk2_y; 
      persistent xp_x;
      persistent xp_y;
      
      persistent SW_sigma_x;
      persistent SW_sigma_y;
      persistent x_wind;
      persistent y_wind;
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
      
      SW_y = 0;
      SW = 0;
      
      SW_sigma_x = 0;
      SW_sigma_y=0;
%       A.Ux = 0;
      
      xp_x = [0 ; 0 ; 0];
      xp_y = [0 ; 0 ; 0];
      y_wind =0;
      x_wind =0;
  end

 
  % dirty derivative of pe to get vhat (error Z )

  
  X_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1 + (2/(2*A.tau+A.Ts))*(A.X_des_EF - errork1);
  X_des_dot = min(3,abs(X_des_dot))*sign(X_des_dot);
  
  X_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2 + (2/(2*A.tau+A.Ts))*(X_des_dot - errork2);
  X_des_ddot = min(6,abs(X_des_ddot))*sign(X_des_ddot);
  
  A.X_dess_dot(A.counter)=X_des_dot;
  A.X_dess_ddot(A.counter)=X_des_ddot;

  error = A.X_des_EF - A.X_kalman;
  A.X_error(A.counter) =  error;
  error_dot = X_des_dot - A.X_dot;
  
  Y_des_dot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk1_y + (2/(2*A.tau+A.Ts))*(A.Y_des_EF - errork1_y);
  Y_des_dot = min(3,abs(Y_des_dot))*sign(Y_des_dot);
  
  Y_des_ddot = (2*A.tau-A.Ts)/(2*A.tau+A.Ts)*vhatk2_y + (2/(2*A.tau+A.Ts))*(Y_des_dot - errork2_y);
  Y_des_ddot = min(6,abs(Y_des_ddot))*sign(Y_des_ddot);
  
  error_y = A.Y_des_EF - A.Y_kalman;
  A.Y_error(A.counter) =  error_y;
  error_dot_y = Y_des_dot - A.Y_dot;
  
    A.X_des_ref(A.counter) = A.X_des_EF;
    A.Y_des_ref(A.counter) = A.X_des_EF;
  

    
%   if A.init==0 
%       xp = [A.Uz*A.Ux/A.m - Kp*(X_des_dot - A.X_des_EF); 0; 0];
% %         xp = [0;0;0];
%   end


flag_control = 1;
  switch(flag_control)  
    case 0
%% 
  % paper Kali
%   
    %Sliding sur face 

      rho = 3;
      rho_y = 3;
      K1_x = 2.5;
      K1_y = 2.5;

      s = error_dot + rho*error;
      s_y = error_dot_y + rho_y*error_y;

      A.s_x(A.counter) = s;
      A.s_y(A.counter) = s_y;
      SW = SW + sat(s)*A.Ts;
      SW_y = SW_y + sat(s_y)*A.Ts;
%       sw_x = SW*sat(s)*A.Ts;
%       sw_y = SW_y*sat(s_y)*A.Ts;


    %   Nonlinear decoupled
      A.theta_des = A.m/A.U1 * (cos(A.psi_kalman)*(-X_des_ddot + rho*error_dot) + sin(A.psi_kalman)*(-Y_des_ddot + rho_y * error_dot_y)) + cos(A.psi_kalman)/A.U1*K1_x*sat(s) + sin(A.psi_kalman)/A.U1*K1_y*sat(s_y);
      A.phi_des   = A.m/A.U1 * (sin(A.psi_kalman)*(-X_des_ddot + rho*error_dot) - cos(A.psi_kalman)*(-Y_des_ddot + rho_y * error_dot_y)) + sin(A.psi_kalman)/A.U1*K1_x*sat(s) - cos(A.psi_kalman)/A.U1*K1_y*sat(s_y);
       
      
%Conclure  GOODDDDDDD !!!!!!!!!!!!
% 

 
    case 1
  %% 
% paper : Novel robust super twisting integral sliding mode controller for a quadrotor under external disturbances
      rho = 3;
      rho_y = 3;
      K1_x = 2.5;
      K1_y = 2.5;

      K2_x = 2;
      K2_y = 2; 
      s = error_dot + rho*error;
      s_y = error_dot_y + rho_y*error_y;

      A.s_x(A.counter) = s;
      A.s_y(A.counter) = s_y;
      
      SW = SW + sat(s)*A.Ts;
      SW_y = SW_y + sat(s_y)*A.Ts;
     
% % SMC
%       Ux =  (X_des_ddot + rho*error_dot + K1_x*sat(s) );
%       Uy =  (Y_des_ddot + rho*error_dot_y + K1_y*sat(s_y));

% % %STSMC
      Uxx =  (X_des_ddot + rho*error_dot + K1_x*sqrt(abs(s))*sat(s) + K2_x*SW );
      Uyy =  (Y_des_ddot + rho*error_dot_y + K1_y*sqrt(abs(s_y))*sat(s_y) + K2_y*SW_y );
%       
     

        
% Observator 
%        u_obs_x =  Uxx - 1*error_dot  ;
%        u_obs_y =  Uyy - 1*error_dot_y  ;
%        
       u_obs_x =  Uxx - rho*error_dot - X_des_ddot  ;
       u_obs_y =  Uyy - rho*error_dot_y - Y_des_ddot;
%        
%        u_obs_x =  K1_x*sqrt(abs(s))*sat(s) + K2_x*SW ;
%        u_obs_y =   K1_x*sqrt(abs(s))*sat(s) +K2_y*SW_y;
       
       A.Uxx_comp(A.counter) = u_obs_x ;
       A.Uxx_comp(A.counter) = u_obs_y ;
%   % new sliding variable for observator   
%        Kp = 1;
%        sigma_x =  A.X_dot + Kp*error ;
%        sigma_y =  A.Y_dot + Kp*error_y  ;
%        
%       SW_sigma_x = SW_sigma_x + sat(sigma_x)*A.Ts;
%       SW_sigma_y = SW_sigma_y + sat(sigma_y)*A.Ts;
%       A.SW_sigma_xx(A.counter) = SW_sigma_x ;
  % PArameter for calulation 
       Phi = [1 A.Ts A.Ts^2/2;0 1 A.Ts;0 0 1];
       c = 4;
  % Calcule differentiator    
    
       [x_0, x_1,x_2] = differentiator(s,xp_x,A.Ts,u_obs_x,Phi,c);
       [y_0, y_1,y_2] = differentiator(s_y,xp_y,A.Ts,u_obs_y,Phi,c);
    
%        [x_0, x_1,x_2] = differentiator(sigma_x,xp_x,A.Ts,u_obs_x,Phi,c);
%        [y_0, y_1,y_2] = differentiator(sigma_y,xp_y,A.Ts,u_obs_y,Phi,c);
%        
       
       xp_x = [x_0 ; x_1;x_2];
       xp_y = [y_0 ; y_1;y_2];
       

    % Plot differentiator 
      A.X_estimate_wind(A.counter) = xp_x(2);
      A.Y_estimate_wind(A.counter) = xp_y(2);
      
      x_wind= (xp_x(2)+x_wind)/2;
      y_wind=(xp_y(2)+y_wind)/2;
     
      A.X_estimate_wind_es(A.counter) = x_wind;
      A.Y_estimate_wind_es(A.counter) = y_wind;
      
      
   
    % New control based on resulte of observator 
    if ( x_wind >1 || y_wind > 1 )
        if x_wind >1
            Ux =  (X_des_ddot + rho*error_dot + 1*sqrt(abs(s))*sat(s) +1*SW -x_wind);
        end
        if y_wind > 1
             Uy =  (Y_des_ddot + rho*error_dot_y + 1*sqrt(abs(s_y))*sat(s_y) +1*SW_y-  y_wind);
        end
        if (x_wind >1 || y_wind > 1)
            Ux =  (X_des_ddot + rho*error_dot + 1*sqrt(abs(s))*sat(s) + SW -x_wind);
            Uy =  (Y_des_ddot + rho*error_dot_y + 1*sqrt(abs(s_y))*sat(s_y) +SW_y- y_wind);
        end
    else
      Ux =  (X_des_ddot+ rho*error_dot + 1*sqrt(abs(s))*sat(s) + 1*SW );
      Uy =  (Y_des_ddot+ rho*error_dot_y + 1*sqrt(abs(s_y))*sat(s_y) + 1*SW_y );
    end
%      if ( xp_x(2)>1 ) 
%       Ux = X_des_ddot + rho*error_dot + 1*abs(s)^(2/3)*sat(s) + 1*abs(s)^(1/3)*SW - xp_x(2);
%       Uy =  Y_des_ddot + rho*error_dot_y + 1*abs(s_y)^(2/3)*sat(s_y) + 1*abs(s_y)^(1/3)*SW_y - xp_y(2);
%      
%      else 
%          Ux = X_des_ddot + rho*error_dot + 1*abs(s)^(2/3)*sat(s) + 1*abs(s)^(1/3)*SW;
%          Uy = Y_des_ddot + rho*error_dot_y + 1*abs(s_y)^(2/3)*sat(s_y) + 1*abs(s_y)^(1/3)*SW_y;
%      end
%       Ux =  error_dot + 2*abs(sigma_x)^(2/3)*sat(sigma_x) + 2*abs(sigma_x)^(1/3)*SW_sigma_x - xp_x(2);
%       Uy =  error_dot_y + 2*abs(sigma_y)^(2/3)*sat(sigma_y) + 2*abs(sigma_y)^(1/3)*SW_sigma_y - xp_y(2);
      
      A.Uxx(A.counter) = Ux;
      A.Ux_obser(A.counter) = Ux - rho*error_dot - X_des_ddot ;
      
      A.Ux_obser_noSW(A.counter) = rho*error_dot + X_des_ddot;
      
% Roll pitch dezire 

      A.theta_des = atan2(Ux*cos(A.psi_kalman) + Uy*sin(A.psi_kalman),A.Uz );
      A.phi_des = atan2(cos(A.theta_kalman)*(Ux*sin(A.psi_kalman) - Uy*cos(A.psi_kalman)),A.Uz);

%         a=Ux/A.Uz;
%         b=Uy/A.Uz;
%         c=cos(A.psi_kalman);
%         d=sin(A.psi_kalman);
%         tan_theta=a*c+b*d;
%         A.theta_des=atan(tan_theta);
% 
%         if A.psi_kalman>=0
%             Psi_ref_singularity=A.psi_kalman-floor(abs(A.psi_kalman)/(2*pi))*2*pi;
%         else
%             Psi_ref_singularity=A.psi_kalman+floor(abs(A.psi_kalman)/(2*pi))*2*pi;
%         end
%         if ((abs(Psi_ref_singularity)<pi/4 || abs(Psi_ref_singularity)>7*pi/4) || (abs(Psi_ref_singularity)>3*pi/4 && abs(Psi_ref_singularity)<5*pi/4))
%             tan_phi=cos(A.theta_des)*(tan(A.theta_des)*d-b)/c;
%         else
%             tan_phi=cos(A.theta_des)*(a-tan(A.theta_des)*c)/d;
%         end
%         A.phi_des=atan(tan_phi);

%   
  
  end
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
