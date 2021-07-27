function script2run

clc 
clear all

global A

figure('units','normalized','position',[.1 .1 .8 .8],'name','Quadrotor AUS','numbertitle','off','color','w');
axes('units','normalized','position',[.2 .1 .6 .8]);
axis equal

%---------- Initilization ---------------------%
All_Variables;

% SimInitial2;         % initilaize the LIDAR
% simLIDARinitial;

quadmodel;
%    Map;


%---------- UIcontrol --------------------------%
% E1 = uicontrol('units','normalized','position',[.11 .85 .1 .07],'style','edit','fontsize',13,'string',0.5,'backgroundcolor','w');
% % E2 = uicontrol('units','normalized','position',[.11 .75 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% % E3 = uicontrol('units','normalized','position',[.11 .65 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% E4 = uicontrol('units','normalized','position',[.11 .55 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% E5 = uicontrol('units','normalized','position',[.11 .45 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
% E6 = uicontrol('units','normalized','position',[.11 .35 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');

% uicontrol('units','normalized','position',[.02 .83 .05 .07],'style','text','fontsize',13,'string','Altitude','backgroundcolor','w');
% % uicontrol('units','normalized','position',[.02 .73 .05 .07],'style','text','fontsize',13,'string','Roll','backgroundcolor','w');
% % uicontrol('units','normalized','position',[.02 .63 .05 .07],'style','text','fontsize',13,'string','Pitch','backgroundcolor','w');
% uicontrol('units','normalized','position',[.02 .53 .05 .07],'style','text','fontsize',13,'string','Yaw','backgroundcolor','w');
% uicontrol('units','normalized','position',[.02 .43 .05 .07],'style','text','fontsize',13,'string','X','backgroundcolor','w');
% uicontrol('units','normalized','position',[.02 .33 .05 .07],'style','text','fontsize',13,'string','Y','backgroundcolor','w');

% uicontrol('units','normalized','position',[.11 .25 .1 .07],'style','pushbutton','fontsize',13,'string','Go','callback',@Go1);

% Motors speed
% uicontrol('units','normalized','position',[.85 .83 .05 .07],'style','text','fontsize',13,'string','Front M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .73 .05 .07],'style','text','fontsize',13,'string','Right M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .63 .05 .07],'style','text','fontsize',13,'string','Rear M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .53 .05 .07],'style','text','fontsize',13,'string','Left M','backgroundcolor',[.5 .7 1]);
% 
% O1 = uicontrol('units','normalized','position',[.91 .86 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
% O2 = uicontrol('units','normalized','position',[.91 .76 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
% O3 = uicontrol('units','normalized','position',[.91 .66 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
% O4 = uicontrol('units','normalized','position',[.91 .56 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');


% disturbances
uicontrol('units','normalized','position',[.13+.77 .35 .08 .07],'style','pushbutton','fontsize',13,'string','Z','callback',@d1);
uicontrol('units','normalized','position',[.02+.77 .35 .08 .07],'style','pushbutton','fontsize',13,'string','Yaw','callback',@d2);
uicontrol('units','normalized','position',[.13+.77 .25 .08 .07],'style','pushbutton','fontsize',13,'string','Pitch','callback',@d3);
uicontrol('units','normalized','position',[.02+.77 .25 .08 .07],'style','pushbutton','fontsize',13,'string','Roll','callback',@d4);
uicontrol('units','normalized','position',[.13+.77 .15 .08 .07],'style','pushbutton','fontsize',13,'string','X','callback',@dx);
uicontrol('units','normalized','position',[.02+.77 .15 .08 .07],'style','pushbutton','fontsize',13,'string','Y','callback',@dy);

% 
% uicontrol('units','normalized','position',[.85 .83 .05 .07],'style','text','fontsize',13,'string','Front M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .73 .05 .07],'style','text','fontsize',13,'string','Right M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .63 .05 .07],'style','text','fontsize',13,'string','Rear M','backgroundcolor',[.5 .7 1]);
% uicontrol('units','normalized','position',[.85 .53 .05 .07],'style','text','fontsize',13,'string','Left M','backgroundcolor',[.5 .7 1]);
% 
% Dx = uicontrol('units','normalized','position',[.85 .83 .05 .07],'style','text','fontsize',13,'string','0','backgroundcolor','w');

pop1 = uicontrol('units','normalized','position',[.02 .15 .19 .07],'style','popupmenu','fontsize',13,'string',{'3D view';'LIDAR Camera';'Camera view';'Top view'},'callback',@view1,'value',1);

axis([-4 4 -4 4 0 8])
view(30,30)
grid on
hold on
%---------- Camera --------------------%


camproj perspective 
camva(25)
set(gcf,'Renderer','OpenGL')

line([-1 1],[0 0],[0 0])
line([0 0],[-.5 .5],[0 0],'color','r')
patch([-10 10 10 -10],[-10 -10 10 10],[0 0 0 0],[0 0 0 0],'facecolor','w')

%---------- starting the loop ----------%
while 1
% tic;  
%---------- measuring the parameters ----------%
   Z_meas;
   XY_meas;
   IMU_meas;
   
%---------- Kalman filters ----------%
    Kalman_phi2;
    Kalman_theta2;
    Kalman_psi2;
    Kalman_Z2;
    Kalman_X2;
    Kalman_Y2;
    
    A.phi_kalman = A.phi_kalman;
    A.theta_kalman = A.theta_kalman;
    A.psi_kalman = A.psi_kalman;
    
    A.X_kalman = A.X;
    A.Y_kalman = A.Y;
    A.Z_kalman = A.Z;
    
%---------- Path manager ----------%
Path_manager;

if A.flagggg==1
    break;
end

%---------- perform PID controllers ----------%
%    PID_X;
%    PID_Y;
%    PID_Z;
%  SMC_U1;
   
%     PID_roll;
%     PID_pitch;
%     PID_heading;
% % %     for i = 1:2 
%     end
    % ALtitude
    SMC_U1;
%       SMC_U1new
% % % Position control
% % % % with observator


% %     SMC_Uxy;
%     SMC_Uxy_2; %% Smoth 

%Robust Trajectory Tracking for Unmanned Aircraft Systems Using High Order Sliding Mode Controllers-Observers
%     SMC_Uxy_3;      % Modifier SWSMC
%     SMC_Uxy_33; % Modifier SWSMC
    
    SMC_Uxy_4; % %  SWSMC - estimed vel and disturbance
%     SMC_Uxy_44;
% 
% %     SMC_Uxy_5;% %  SWSMC - estimed only disturbance
% % % No observator 

%     SMC_Uxy_feedfoward                % Conclure : Not work 
%     SMC_Uxy_normal
% %     %Attitude 
% % %     for i = 1:2 
    SMC_attitude;
%     


%     end

%----------- limit the motors speed ----------%   
%     for i = 1:2 
        Motors_Speed_2;
        Forces;
%---------- apply the equations of motion -----------%
        quadmodel;
    
    
%----------- Simulate the LIDAR ------------------%
%    Sim_LIDAR;       
   
%---------- Calculating Motors speed and displaying it ------------%

%    set(O1,'string',num2str(A.O1));
%    set(O2,'string',num2str(A.O2));
%    set(O3,'string',num2str(A.O3));
%    set(O4,'string',num2str(A.O4));
% 
%    set(Dx,'string',num2str(A.wind_force_x));
%    set(O2,'string',num2str(A.O2));
%    set(O3,'string',num2str(A.O3));
%    set(O4,'string',num2str(A.O4));
   
%---------- PLotting the quadrotor new position ----------%
       if(A.flag==3)
          plot_quad_3D;         % update the position and orientation of the quadrotor
%           LIDAR_Plot            % update the LIDAR beams
    
          switch(A.Camera_View)
              case 0
          campos([A.X+4 A.Y-2 A.Z+3])
          camtarget([A.X A.Y A.Z])
          camroll(0);

              case 1
          campos([A.X+6 A.Y-6 A.Z+8])
          camtarget([A.X A.Y A.Z])
          camroll(0);

              case 2
          campos([A.X A.Y A.Z])
          camtarget([A.X+A.Z*sin(A.theta) A.Y+A.Z*sin(A.phi) 0])
          camroll(A.psi*180/pi-A.psi_cam)
          A.psi_cam = A.psi*180/pi;

              case 3
          campos([A.X A.Y+1 A.Z+13])
          camtarget([A.X A.Y A.Z])
          camroll(0);

          end
          drawnow
  
          A.flag=0;
%        toc;
%           while(toc<.03)
%           end
       end
       A.flag = A.flag+1;
       A.init = 1;  % stop the initilization in my code
      
%     end
end




%---------- Subfunctions -----------%

%     function Go1(varargin)
%         A.Z_des = str2double(get(E1,'string'));
% %         A.phi_des = str2double(get(E2,'string'))*pi/180;
% %         A.theta_des = str2double(get(E3,'string'))*pi/180;
%         A.psi_des = str2double(get(E4,'string'))*pi/180;
%         A.X_des_EF = str2double(get(E5,'string'));
%         A.Y_des_EF = str2double(get(E6,'string'));
%     end

% disturbances sub functions
    function d1(varargin)   % disturbance in Z
        A.Z_dis = 5;
    end

    function d2(varargin)   % disturbance in Yaw
        A.psi_dis = -200*A.Izz;
    end

    function d3(varargin)   % disturbance in Pitch
        A.theta_dis = -200*A.Iyy;
    end

    function d4(varargin)   % disturbance in Roll
        A.phi_dis = -200*A.Ixx;
    end
    function dx(varargin)   % disturbance in x
            A.X_dis = A.X_diss;
    end
    function dy(varargin)   % disturbance in y
            A.Y_dis = A.Y_diss;
    end

    function view1(varargin)
       A.var1 = get(pop1,'value');
       switch A.var1
           case 1
               A.Camera_View = 0;
           case 2
               A.Camera_View = 1;
           case 3
               A.Camera_View = 2;
           case 4
               A.Camera_View = 3;
       end
    end
    save('A','A');
end
