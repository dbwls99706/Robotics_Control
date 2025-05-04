%%
clc
clear all
close all

%시뮬레이션 변수 설정
flag_sim = 1;
flag_draw = 1;
flag_draw_robot = 1;
flag_draw_graph = 1;

global Iz1 Iz2 Iz3 L1 L2 L3 r1 r2 r3 g m1 m2 m3 tau1 tau2 tau3;

%시뮬레이션 시간 설정
dt = 0.005;             % 샘플링
st = 0.000;             % 시작
ft = 5.000;             % 종료

g = 9.8148;             

% 주어진 로봇 파라미터
m1 = 1.0;   m2 = 1.0;   m3 = 1.0;                       %막대기 질량
L1 = 1.0;   L2 = 1.0;   L3 = 1.0;                       %링크 길이
r1 = L1;    r2 = L2;    r3 = L3;                        %질량 중심
Iz1 = (m1*L1^2)/3; Iz2 = (m2*L2^2)/3; Iz3 = (m3*L3^2)/3;%링크 관성
tau1 = 0.0; tau2 = 0.0; tau3 = 0.0;                     %토크 변수

%계산용 변수 초기화
init_q1 = 0; init_q2 = 0; init_q3 = 0;                  
init_dq1 = 0.00; init_dq2 = 0.00; init_dq3 = 0.00;      
q = [init_q1; init_q2; init_q3];                        
dq = [init_dq1; init_dq2; init_dq3];                    

% 타겟 파라미터
q_d = [init_q1; init_q2; init_q3];                      % 각도
dq_d = [0; 0; 0];                                       % 각속도
ddq_d = [0; 0; 0];                                      % 각가속도

% 적분기용 변수
q_err_sum = 0;          

% 제어기 수치
Wn = 30;                % 제어주기
Kp = Wn^2;              % P 게인
Kv = 2*Wn;              % D 게인
Ki = 150;               % I 게인

if (flag_sim == 1)
    n = 1;
    for time = st:dt:ft        
        if (time < 1)
            q_d = [init_q1; init_q2; init_q3];
            dq_d = [0; 0; 0];
            ddq_d = [0; 0; 0];
        else
            if (q_d(1) < 90*pi/180)
                q_d(1) = q_d(1) + (30*pi/180)*dt;
            else
                q_d(1) = 90*pi/180;
            end
            if (q_d(2) < 60*pi/180)
                q_d(2) = q_d(2) + (30*pi/180)*dt;
            else
                q_d(2) = 60*pi/180;
            end
            if (q_d(3) < 30*pi/180)
                q_d(3) = q_d(3) + (30*pi/180)*dt;
            else
                q_d(3) = 30*pi/180;
            end
            dq_d(1) = (q_d(1) - save_q1_d(n-1))/dt;
            ddq_d(1) = (dq_d(1) - save_dq1_d(n-1))/dt;
            dq_d(2) = (q_d(2) - save_q2_d(n-1))/dt;
            ddq_d(2) = (dq_d(2) - save_dq2_d(n-1))/dt;
            dq_d(3) = (q_d(3) - save_q3_d(n-1))/dt;
            ddq_d(3) = (dq_d(3) - save_dq3_d(n-1))/dt;
        end
        
        % 기구학 적용
        D = three_link_I(q(1), q(2), q(3));                        % 관성 계산
        G = three_link_G(q(1), q(2), q(3));                        % 중력 계산
        
        % 계산
        q_err_sum = q_err_sum + (q_d-q)*dt;                         % 적분기
        u = ddq_d + Kv*(dq_d - dq) + Kp*(q_d - q) + Ki*q_err_sum;	% PID 제어기
        gravity_err = 1.2;                                          % 중력오차수정
        tq_ctrl = D*u + G*gravity_err;                              % 조인트별 토크 컨트롤

        %역기구학
        tau1 = tq_ctrl(1);
        tau2 = tq_ctrl(2);
        tau3 = tq_ctrl(3);
        
        % 미분방정식 적용
        [t, y] = ode45('three_link', [0 dt], [q(1); dq(1); q(2); dq(2); q(3); dq(3)]);
        index = length(y);
        
        % 각 링크별 상태 반환
        q = [y(index, 1); y(index, 3); y(index, 5)];
        dq = [y(index, 2); y(index, 4); y(index, 6)];
        
        % Save data
        save_time(n) = time;         % [sec]
        save_q1(n) = q(1);           % [rad]
        save_q2(n) = q(2);           % [rad]
        save_q3(n) = q(3);           % [rad]
        save_dq1(n) = dq(1);         % [rad/s]
        save_dq2(n) = dq(2);         % [rad/s]
        save_dq3(n) = dq(3);         % [rad/s]
        save_q1_d(n) = q_d(1);       % [rad], 
        save_q2_d(n) = q_d(2);       % [rad], 
        save_q3_d(n) = q_d(3);       % [rad], 
        save_dq1_d(n) = dq_d(1);     % [rad/s], 
        save_dq2_d(n) = dq_d(2);     % [rad/s], 
        save_dq3_d(n) = dq_d(3);     % [rad/s], 
        n = n + 1;
    end
end

%% Simulation result graph
if (flag_draw == 1)
    font_size_label = 20;
    font_size_title = 25;
    linewidth_current = 3;
    linewidth_target = 5;
    
    if (flag_draw_robot == 1)
        % Draw robot
        % save as gif
        filename = 'joint_3dof.gif';
        
        % Init robot position
        x1 = L1*sin(init_q1);
        y1 = -L1*cos(init_q1);
        x2 = L2*sin(init_q1 + init_q2);
        y2 = -L2*cos(init_q1 + init_q2);
        x3 = L3*sin(init_q1 + init_q2 + init_q3);
        y3 = -L3*cos(init_q1 + init_q2 + init_q3);
        
        FG1 = figure('Color', [1 1 1]);
        AX = axes('parent', FG1);
        hold on;
        
        Px1 = [0 x1];               Py1 = [0 y1];
        Px2 = [x1 x1+x2];           Py2 = [y1 y1+y2];
        Px3 = [x1+x2 x1+x2+x3];     Py3 = [y1+y2 y1+y2+y3];
        
        p1 = plot(Px1, Py1, '-or', 'Linewidth', linewidth_current);
        p2 = plot(Px2, Py2, '-og', 'Linewidth', linewidth_current);
        p3 = plot(Px3, Py3, '-ob', 'Linewidth', linewidth_current);
                
        axis([-3.0 3.0 -3.0 3.0]);
        grid on;
        xlabel('X-axis (m)', 'fontsize', font_size_label);
        ylabel('Y-axis (m)', 'fontsize', font_size_label);
        title('3-DOF Robot', 'fontsize', font_size_title);
        
        n = 1;
        for (time = st:dt:ft)
            % Print run time
            cmd = sprintf("Time: %2.2f", time);
            clc
            disp(cmd);
            
            q1 = save_q1(n);                             % [deg], joint 1 angle
            q2 = save_q2(n);                             % [deg], joint 2 angle
            q3 = save_q3(n);                             % [deg], joint 3 angle
            x1 = L1*sin(q1);        y1 = -L1*cos(q1);           % [m], joint 1 position
            x2 = L2*sin(q1+q2);     y2 = -L2*cos(q1+q2);        % [m], joint 2 position
            x3 = L3*sin(q1+q2+q3);	y3 = -L3*cos(q1+q2+q3);     % [m], joint 3 position
            
            Px1 = [0 x1];               Py1 = [0 y1];
            Px2 = [x1 x1+x2];           Py2 = [y1 y1+y2];
            Px3 = [x1+x2 x1+x2+x3];     Py3 = [y1+y2 y1+y2+y3];
            set(p1, 'XData', Px1, 'YData', Py1);
            set(p2, 'XData', Px2, 'YData', Py2);
            set(p3, 'XData', Px3, 'YData', Py3);
            drawnow
            n = n + 1;
            
           
        end
    end
    
    if (flag_draw_graph == 1)
        % Draw angle
        FG2 = figure('Color', [1 1 1]);
        % joint 1
        plot(save_time, save_q1_d*180/pi, ':r', 'LineWidth', linewidth_target); hold on;
        plot(save_time, save_q1*180/pi, 'r', 'LineWidth', linewidth_current); hold on;
        % joint 2
        plot(save_time, save_q2_d*180/pi, ':g', 'LineWidth', linewidth_target-1); hold on;
        plot(save_time, save_q2*180/pi, 'g', 'LineWidth', linewidth_current-1); hold on;
        % joint 3
        plot(save_time, save_q3_d*180/pi, ':b', 'LineWidth', linewidth_target-2); hold on;
        plot(save_time, save_q3*180/pi, 'b', 'LineWidth', linewidth_current-2); hold on;
        
        axis([st ft 0 120]);
        grid on;
        
        xlabel('time (s)', 'fontsize', font_size_label);
        ylabel('Angle (deg)', 'fontsize', font_size_label);
        title('Joint Space PID CTM Controller', 'fontsize', font_size_title);
        legend('tar_{q1}', 'cur_{q1}', 'tar_{q2}', 'cur_{q2}', 'tar_{q3}', 'cur_{q3}', 'Location', 'best');
        
        % Draw angular velocity
        FG3 = figure('Color', [1 1 1]);
        % joint 1
        plot(save_time, save_dq1_d*180/pi, ':r', 'LineWidth', linewidth_target); hold on;
        plot(save_time, save_dq1*180/pi, 'r', 'LineWidth', linewidth_current); hold on;
        % joint 2
        plot(save_time, save_dq2_d*180/pi, ':g', 'LineWidth', linewidth_target-1); hold on;
        plot(save_time, save_dq2*180/pi, 'g', 'LineWidth', linewidth_current-1); hold on;
        % joint 3
        plot(save_time, save_dq3_d*180/pi, ':b', 'LineWidth', linewidth_target-2); hold on;
        plot(save_time, save_dq3*180/pi, 'b', 'LineWidth', linewidth_current-2); hold on;
        
        axis([st ft -90 90]);
        grid on;
        
        xlabel('time (s)', 'fontsize', font_size_label);
        ylabel('Velocity (deg/s)', 'fontsize', font_size_label);
        title('Joint Space PID CTM Controller', 'fontsize', font_size_title);
        legend('tar_{dq1}', 'cur_{dq1}', 'tar_{dq2}', 'cur_{dq2}', 'tar_{dq3}', 'cur_{dq3}', 'Location', 'best');
    end
end
