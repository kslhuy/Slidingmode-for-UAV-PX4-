function derivatif = dirtyderivatif(y,Ts,tau)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global A
persistent vhatk1;
persistent errork1;

if A.init ==0
 vhatk1 = 0;
 errork1 = 0;
end 
dy = (2*tau-Ts)/(2*tau+Ts)*vhatk1 + (2/(2*tau+Ts))*(y - errork1);

vhatk1 = dy;
errork1 = y;
derivatif = dy;

end

