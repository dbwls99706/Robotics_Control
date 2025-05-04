clc,clear all 
close all

% 시뮬레이션 세팅
flag_sim=1;
flag_draw=1;
flag_draw_robot= 1;
flag_draw_graph= 1;

global Iz1 Iz2 L1 L2 g m1 m2 r1 r2 tq1 tq2;

% 시간
dt = 0.005;             % 샘플링
st = 0.000;             % 시작
ft = 5.000;             % 종료

g = 9.8148;             % 중력가속도

%로봇 파라미터
m1 = 0.2;       m2 = 0.2;       % 막대기 질량
L1 = 0.5;       L2 = 0.5;       % 길이
r1 = 0.1;       r2 = 0.1;       % 질량중심
Iz1 = 0.05;     Iz2 = 0.05;     % 관성

init_q1 = -pi/2; init_q2 = pi/2;       %rad
init_dq1 = 0.0;  init_dq2 = 0.00;      %rad/s
q = [init_q1; init_q2];                %rad
dq = [init_dq1; init_dq2];             %rad/s

pre_J = zeros(2,2);

init_X = get_Kinematics(q(1), q(2));     % 엔드 이펙터 초기화
X = init_X;                             % 현재 위치
dX = [0; 0];                            % 현재 속도
X_d = init_X;                           % 타겟 위치 엔드 이펙터
dX_d = [0; 0];                          % 타겟 속도 엔드 이펙터
ddX_d = [0; 0];                         % 타겟 가속도 엔드 이펙터

pre_X = 0;
dX_d_old = [0;0];    
dX_d_int = [0;0];

tq1 = 0.0;
tq2 = 0.0;
tq = [tq1;tq2];                         % 토크 컨트롤

%Controller Gain
Wn = 20;                % 고유 진동수
Kp = Wn^2;              % P gain
Kv = 2*Wn;              % D gain
Ki = 500;               % I gain
%% Simulation
if(flag_sim == 1)
    %Simulation
        n = 1;
        sin_t = 0;
        for(time = st:dt:ft)
        %Set Target Position  
            if(time < 1.0)
                X_d         = init_X;
                dX_d        = [0;0]; 
                ddX_d       = [0;0];
            elseif(time < 2.0)
                X_d(1)      = init_X(1);
                if(X_d(2) < init_X(2) + 0.1)
                    X_d(2) = X_d(2) + (0.1/0.5)*dt;
                else
                    X_d(2) = init_X(2) + 0.1;
                end
                dX_d = (X_d - [simul_X_d_x(n-1); simul_X_d_y(n-1)])./dt;
                ddX_d = (dX_d - [simul_dX_d_x(n-1); simul_dX_d_y(n-1)])./dt;
            else
                X_d     =[ 0.1 * sin((2*pi*sin_t)/2) + init_X(1);
                           0.1 * cos((2*pi*sin_t)/2) + init_X(2);];
                sin_t = sin_t + dt;
                dX_d    = (X_d - [simul_X_d_x(n-1); simul_X_d_y(n-1)])./dt;
                ddX_d   = (dX_d - [simul_dX_d_x(n-1); simul_dX_d_y(n-1)])./dt;
            end
                
                %Get Dynamics
                J = get_Jacobian(q(1),q(2));            %자코비안
                dJ = (J - pre_J)/dt;
                pre_J = J; 

                X = get_Kinematics(q(1),q(2));          %위치
                dX = J*dq;                              %속도

                D = get_Inertia(q(2));                  %관성
                H = get_Coriollis(q(2),dq(1),dq(2));    %전향력
                C = get_Gravity(q(1),q(2));             %중력
                
                % For error function
                dX_d_int = dX_d_int + (dX_d - dX_d_old)*dt;
                dX_d_old = dX_d;

                %Control
                u = ddX_d+Kv*(dX_d - dX) + Kp*(X_d - X) + Ki*dX_d_int;    %PID제어
                
                ddq_ref = inv(J)*(u - dJ*dq);

                tq_ctrl = D*ddq_ref + H + C*0.8;

                % Inverse
                tq = tq_ctrl;
                tq1 = tq(1);    tq2 = tq(2);
                [t,y] = ode45('two_links', [0 dt], [q(1); dq(1); q(2); dq(2)]);
                index = length(y);
                q = [y(index, 1); y(index,3)];
                dq = [y(index,2); y(index,4)];

                %save Data
                simul_time(n)   = time;             %sec
                simul_q1(n)      = q(1);            %rad
                simul_q2(n)      = q(2);            %rad
                simul_dq1(n)     = dq(1);           %rad/s
                simul_dq(n)      = dq(2);           %rad/s

                simul_X_x(n)     = X(1);            %m
                simul_X_y(n)     = X(2);            %m

                simul_dX_x(n)     = dX(1);          %m/s
                simul_dX_y(n)     = dX(2);          %m/s

                simul_X_d_x(n)    = X_d(1);         %m
                simul_X_d_y(n)    = X_d(2);         %m

                simul_dX_d_x(n)   = dX_d(1);        %m/s
                simul_dX_d_y(n)   = dX_d(2);        %m/s

                n                 = n+ 1;
        end
end
            
    
    %% Simulation Result Graph
        if(flag_draw == 1)
            fontsize           =20;
            font_size_title    =25;
            font_size_label    =20;
            linewidth_current   =3;
            linewidth_target    =5;
            
            if(flag_draw_robot == 1)
                % Draw Robot
                    x1 = L1*cos(init_q1);
                    y1 = L1*sin(init_q1);
                    x2 = L2*cos(init_q1+init_q2);
                    y2 = L2*sin(init_q1+init_q2);
                    
                    FG1 = figure('Position',[200 300 700 700], 'Color', [1 1 1]);
                        AX = axes('parent',FG1); hold on
                        
                        Px1 = [0  x1];
                        Py1 = [0  y1];
                        Px2 = [x1 x1+x2];
                        Py2 = [y1 y1+y2];
                        
                        p1 = plot(Px1,Py1, '-ob', 'Linewidth', linewidth_current);
                        p2 = plot(Px2,Py2, '-or', 'Linewidth', linewidth_current);
                        
                        axis([-1.0 1.0 -1.6 0.4]);
                        grid on
                    xlabel('X-axis (m)',    'fontsize', font_size_label)
                    ylabel('Y-axis (m)',    'fontsize', font_size_label)
                    title( '2-DOF Robot',   'fontsize', font_size_title)
                    
                    
                    n = 1;
                    for(time=st:dt:ft)
                        q1 = simul_q1(n);
                        q2 = simul_q2(n);
                        
                        x1 = L1*cos(q1);
                        y1 = L1*sin(q1);
                        x2 = L2*cos(q1+q2);
                        y2 = L2*sin(q1+q2);
                        
                        Px1 = [0  x1];
                        Py1 = [0  y1];
                        Px2 = [x1 x1+x2];
                        Py2 = [y1 y1+y2];
                        
                        set(p1, 'XData', Px1, 'YData', Py1)
                        set(p2, 'XData', Px2, 'YData', Py2)
                        drawnow
                        n = n+1;
                    end
            end
            
            if(flag_draw_graph == 1)
                %위치
                FG2 = figure('Position',[900 700 600 300],'Color',[1 1 1]);
                
                    plot(simul_time,simul_X_d_x,':r','linewidth',linewidth_target); hold on;
                    plot(simul_time,simul_X_d_y,':b','linewidth',linewidth_target); hold on;
                    
                    plot(simul_time,simul_X_x,'r','linewidth',linewidth_current); hold on;
                    plot(simul_time,simul_X_y,'b','linewidth',linewidth_current); hold on;
                    
                    axis([st ft -1.25 1]);
                    xticks([st:1:ft])
                    yticks([-1:0.25:1])
                    grid on
                    
                    legend({'tar_x', 'tar_y','cur_x','cur_y'},'location','best','orientation','horizontal','fontsize',15)
                    
                    xlabel('time (s)',              'fontsize',font_size_label)
                    ylabel('Position (m)',           'fontsize', font_size_label)
                    title( 'Cartesin Space PID CTM Controller', 'fontsize', font_size_title)
                    
                %속도
                FG3 = figure('Position',[900 300 600 300],'Color',[1 1 1]);
                
                    plot(simul_time,simul_dX_d_x,':r','linewidth',linewidth_target); hold on;
                    plot(simul_time,simul_dX_d_y,':b','linewidth',linewidth_target); hold on;
                    
                    plot(simul_time,simul_dX_x,'r','linewidth',linewidth_current); hold on;
                    plot(simul_time,simul_dX_y,'b','linewidth',linewidth_current); hold on;
                    
                    axis([st ft -1.25 1.25]);
                    xticks([st:1:ft])
                    yticks([-1.25:0.25:1.25])
                    grid on
                    
                    legend({'tar_x', 'tar_y','cur_x','cur_y'},'location','best','orientation','horizontal','fontsize',15)
                    
                    xlabel('time (s)',              'fontsize',font_size_label)
                    ylabel('Velocity (m/s)',           'fontsize', font_size_label)
                    title( 'Cartesin Space PID CTM Controller', 'fontsize', font_size_title)
               
            end
        end
        
 



        