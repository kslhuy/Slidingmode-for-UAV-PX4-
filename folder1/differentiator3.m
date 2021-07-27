function [x_1,x_2,x_3,e1] = differentiator3(f , xp , tau,u_obes,lamda1z,lamda2z,lamda3z)

   %compute error 
    sigma_0 =f - xp(1) ; % sigma_0 ... estimation error
    
    xp(1) = xp(1) + tau*lamda1z *abs(sigma_0)^(2/3)*sign(sigma_0) + (tau*xp(2) + (tau^(2)/2)*xp(3));
    xp(2) = xp(2) + tau*lamda2z*abs(sigma_0)^(1/3)*sign(sigma_0) + (tau*xp(3)) +  tau*(u_obes);
    e1 = xp(3) + tau*lamda3z*sign(sigma_0) ;
    xp(3) = xp(3) + tau*lamda3z*sat(sigma_0/0.002) ;
    
    % Compute Estimates 
    x_1 = xp(1);
    x_2 = xp(2);
    x_3 = xp(3);
    
end