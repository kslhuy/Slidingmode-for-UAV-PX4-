global A
fig = figure;
axes('fontsize',16)
% subplot(2,1,1);
% 
% % cla
hold on
% % plot(A.t_plot(1:A.counter),A.X_plot(1:A.counter)+A.X_error(1:A.counter),'y')
plot(ACopy.t_plot(1:A.counter-1),ACopy.X_plot_dis(1:A.counter-1),'b','linewidth',2)
plot(ACopy.t_plot(1:A.counter-1),ACopy.dis_x_es(1:A.counter-1),'m','linewidth',2)
% % plot(A.t_plot(1:A.counter-1),A.X_ref_plot(1:A.counter-1),'b','linewidth',2)
% % plot(A.t_plot(1:A.counter),A.X_dis_plot(1:A.counter),'g')
% % plot(A.t_plot(1:A.counter),A.X_kalman_plot(1:A.counter),'color',[1 .4 .8])
% a=legend('dis_{ref} ','dis_{est_sat}');
% axis([0 55 -1 4])
% set(a,'fontsize',12)
% grid on
% % xlabel('Time (s)','fontsize',12)
% % ylabel('X axis (meter)','fontsize',12)
% % title('X axis vs. time','fontsize',16)
% hold off
% subplot(2,1,2);

% hold on
% plot(A.t_plot(1:A.counter),A.X_plot(1:A.counter)+A.X_error(1:A.counter),'y')
% plot(A.t_plot(1:A.counter-1),A.X_plot_dis(1:A.counter-1),'b','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.dis_x_es(1:A.counter-1),'m','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.dis_x_es_fil(1:A.counter-1),'r','linewidth',2)
% plot(A.t_plot(1:A.counter-1),A.X_ref_plot(1:A.counter-1),'b','linewidth',2)
% plot(A.t_plot(1:A.counter),A.X_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.X_kalman_plot(1:A.counter),'color',[1 .4 .8])
a=legend('dis_{ref} ','dis_{est+sign}','dis_{est+sign+lowpass}');
set(a,'fontsize',12)
axis([0 55 -1 4])
grid on

% han=axes(fig,'visible','off'); 
% han.Title.Visible='on';
% han.XLabel.Visible='on';
% han.YLabel.Visible='on';
% ylabel(han,'Disturbance','fontsize',16);
% xlabel(han,'Time (s)','fontsize',16);

% % title(han,'yourTitle');

xlabel('Time (s)','fontsize',15)
ylabel('Disturbance','fontsize',16)
% title('X axis vs. time','fontsize',16)