% this function will plot the Z value with the desired one. 
global A
figure
% cla
axes('fontsize',16)
hold on

plot3(ACopy.X_plot(1:A.counter-1),ACopy.Y_plot(1:A.counter-1),ACopy.Z_plot(1:A.counter-1),'linewidth',2)
plot3(ACopy.X_path,ACopy.Y_path,ACopy.Z_path,'r','linewidth',2) 
A.t_plot = [0.01:0.01:A.counter*0.01];


view(70,40)
grid on
% axis equal
axis([-4 2.5 -2.5 2.5 0 2.5])

% figure(2)
% hold on
% plot(A.t_plot(1:A.counter),A.Y_plot(1:A.counter),'r','linewidth',1)
% plot(A.t_plot(1:A.counter),A.Y_ref_plot(1:A.counter),'b')
% plot(A.t_plot(1:A.counter),A.Y_dis_plot(1:A.counter),'g')
% % 
% % for i=1:A.num_obstacles(1)
% % patch([A.B1(i,1) A.C1(i,1) A.C1(i,1) A.B1(i,1)],[A.B1(i,2) A.C1(i,2) A.C1(i,2) A.B1(i,2)],[A.B1(i,3) A.B1(i,3) A.D1(i,3) A.D1(i,3)],[0 .4 1])
% % end
% % 
a=legend('Quadrotor path','Desired path');
set(a,'fontsize',12)

xlabel('X axis (m)','fontsize',16)
ylabel('Y axis (m)','fontsize',16)
zlabel('Z axis (m)','fontsize',16)
hold off

% title('path traveled','fontsize',16)
% hold off