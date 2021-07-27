% clc 
% clear 
% init = 0 ;
% t =0:1e-3:10;
% f = sin(t)+5*t ;
% f_dot = cos(t) + 5;
% ode = diff(f,1);
% Ts =10^-4;
% lamda0 = 5;
% lamda1 =8;
% y = f;
% x(1) = 0 ;
% U_1(1) = 0;
% d_y(1) = 0;
% tau = 0.1;
% % code
% for k=2:length(f)
%     x(k) = x(k-1)+Ts*d_y(k-1);       %Integration of X (internal variable) using Euler backward method (x-dot= u)
%     u_1(k) = (-lamda1)*sign(x(k)-y(k-1)); % Formula of u_1 dot in the algorithm 
%     U_1(k) = U_1(k-1)+u_1(k);  % Integration of u_1 dot to obtain u_1 using Euler forward method 
%     d_y(k) = U_1(k) - lamda0*sqrt(abs(x(k)-y(k-1)))*sign(x(k)-y(k-1)); % derivative of y (U in the algorithm)
% %     dy(k)=(y(k)-y(k-1))/Ts; % Derivative of Y using differece formula
% 
% end
% for k=1:length(f)
%     dy(k) = dirtyderivatif(y(k) , Ts , tau , init);
%     init = 1;
% end
% %   end
% 
% plot (t,f_dot,'r')
% hold on
% plot (t,dy)
% % hold on 
% % plot (t , U_1,'blue')
% % plo
T = 0.01
phi = (M*T)^0/factorial(0) + (M*T)^1/factorial(1) + (M*T)^2/factorial(2) 