global A
fig = figure;
axes('fontsize',16)
subplot(3,1,1);
% 
% % cla
hold on
% % plot(A.t_plot(1:A.counter),A.X_plot(1:A.counter)+A.X_error(1:A.counter),'y')
plot(A.t_plot(1:A.counter-1),A.Uxx(1:A.counter-1),'b','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.x_dot_es(1:A.counter-1),'m','linewidth',2)
% % plot(A.t_plot(1:A.counter-1),A.X_ref_plot(1:A.counter-1),'b','linewidth',2)
% % plot(A.t_plot(1:A.counter),A.X_dis_plot(1:A.counter),'g')
% % plot(A.t_plot(1:A.counter),A.X_kalman_plot(1:A.counter),'color',[1 .4 .8])
% a=legend('velX_{rep} ','velX_{obs}');
% axis([0 55 -1 4])
axis([0 52 -4 1])
% set(a,'fontsize',12)
% grid on
% xlabel('Time (s)','fontsize',12)
ylabel('U_X ','fontsize',15)
% % title('X axis vs. time','fontsize',16)
grid on
hold off
title('Control signal','fontsize',14)
subplot(3,1,2);

hold on
plot(A.t_plot(1:A.counter-1),A.Uyy(1:A.counter-1),'r','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.y_dot_es(1:A.counter-1),'m','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.dis_x_es(1:A.counter-1),'m','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.dis_x_es_fil(1:A.counter-1),'r','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.X_ref_plot(1:A.counter-1),'b','linewidth',2)
% plot(A.t_plot(1:A.counter),A.X_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.X_kalman_plot(1:A.counter),'color',[1 .4 .8])
% a=legend('Ux{rep} ','velY_{obs}');
% set(a,'fontsize',12)
axis([0 52 -4 1])
grid on

% han=axes(fig,'visible','off'); 
% han.Title.Visible='on';
% han.XLabel.Visible='on';
% han.YLabel.Visible='on';
% ylabel(han,'Disturbance','fontsize',16);
% xlabel(han,'Time (s)','fontsize',16);

% % title(han,'yourTitle');
hold off
% xlabel('Time (s)','fontsize',15)
ylabel('U_Y ','fontsize',15)
subplot(3,1,3);

hold on
plot(A.t_plot(1:A.counter-1),A.Uzz(1:A.counter-1),'m','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.y_dot_es(1:A.counter-1),'m','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.dis_x_es(1:A.counter-1),'m','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.dis_x_es_fil(1:A.counter-1),'r','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.X_ref_plot(1:A.counter-1),'b','linewidth',2)
% plot(A.t_plot(1:A.counter),A.X_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.X_kalman_plot(1:A.counter),'color',[1 .4 .8])
% a=legend('Ux{rep} ','velY_{obs}');
% set(a,'fontsize',12)
axis([0 52 0 10])
grid on

% han=axes(fig,'visible','off'); 
% han.Title.Visible='on';
% han.XLabel.Visible='on';
% han.YLabel.Visible='on';
% ylabel(han,'Disturbance','fontsize',16);
% xlabel(han,'Time (s)','fontsize',16);

% % title(han,'yourTitle');
hold off
xlabel('Time (s)','fontsize',15)
ylabel('U_Z ','fontsize',15)
