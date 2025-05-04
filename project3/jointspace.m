clear all
close all

% 시뮬레이션 세팅
flag_sim = 1;
flag_draw = 1;
flag_draw_robot = 1;
flag_draw_graph = 1;

global m I L g tau;

% 시간
dt = 0.005;             % 샘플링
st = 0.000;             % 시작
ft = 5.000;             % 종료

g = 9.8148;             % 중력가속도

%로봇 파라미터
m = 1.0000;             % 막대기 질량
L = 1.0000;             % 길이
I = (m*L^2)/3;          % 관성
tau = 0.0000;           % 토크제어용 변수

%초기 변수 선언
q = 0;            % 각도
dq = 0;           % 각속도

% 타겟 변수 선언
q_d = 0;                % 각도
dq_d = 0;               % 각속도
ddq_d = 0;              % 각가속도

% 적분기 변수
q_err_sum = 0;          

% 제어기 변수
Wn = 20;                % 주기
Kp = Wn^2;              % P gain
Kv = 2*Wn;              % D gain
Ki = 250;               % I gain

if (flag_sim == 1)
    n = 1;
    for time = st:dt:ft
        
        if (time < 1)
            q_d = 0;
            dq_d = 0.0;
            ddq_d = 0.0;
        else
            if (q_d < 90*pi/180)
                q_d = q_d + (30*pi/180)*dt;
            else
                q_d = 90*pi/180;
            end
            % Stay angular velocity to 30 deg/s
            dq_d = (q_d - save_q_d(n-1))/dt;
            ddq_d = (dq_d - save_dq_d(n-1))/dt;
        end
        % 계산
        G = joint_gravity(q);
        % Controller
        q_err_sum = q_err_sum + (q_d-q)*dt;                         % 적분기
        u = ddq_d + Kv*(dq_d - dq) + Kp*(q_d - q) + Ki*q_err_sum;	% PID 컨트롤러
        gravity_err = 1.2;                                          % 중력 오차 수정
        tq_ctrl = I*u + G*gravity_err;                              % 각각에 대한 토크 컨트롤

        %역기구학
        tau = tq_ctrl;
        
        % 링크 상태 반환하기
        [t, y] = ode45('one_link', [0 dt], [q; dq]);
        index = length(y);
        
        % 현재 정보 수정
        q = y(index, 1);
        dq = y(index, 2);
        
        save_time(n) = time;     % 초
        save_q(n) = q;           % rad
        save_dq(n) = dq;         % rad/s
        save_q_d(n) = q_d;       % 타겟 각도
        save_dq_d(n) = dq_d;     % 타겟 속도
        n = n + 1;
    end
end

if (flag_draw == 1)
    label_size = 20;
    title_size = 25;
    c_line = 3;
    t_line = 5;
    
    if (flag_draw_robot == 1)
        % Draw robot
        % save as gif
        filename = 'jointspace.gif';
        
        % Init robot position
        init_x = L*sin(0);
        init_y = -L*cos(0);
        
        FG1 = figure('Color', [1 1 1]);
        AX = axes('parent', FG1);
        hold on;
        
        p = plot([0 0], [init_x init_y], '-ob', 'Linewidth', c_line);
        
        axis([-1.5 1.5 -1.5 1.5]);
        grid on;
        xlabel('X-axis (m)', 'fontsize', label_size);
        ylabel('Y-axis (m)', 'fontsize', label_size);
        title('1-DOF', 'fontsize', title_size);
        
        n = 1;
        for (time = st:dt:ft)

            q = save_q(n);
            x = L*sin(q);   y = -L*cos(q);
            Px = [0, x];    Py = [0, y];
            set(p, 'XData', Px, 'YData', Py);
            drawnow
            n = n + 1;
            

        end
    end
    
    if (flag_draw_graph == 1)
        % Draw angle
        FG2 = figure('Color', [1 1 1]);
        plot(save_time, save_q_d*180/pi, ':k', 'LineWidth', t_line);
        hold on;
        plot(save_time, save_q*180/pi, 'r', 'LineWidth', c_line);
        hold on;
        
        axis([st ft 0 120]);
        grid on;
        
        xlabel('time (s)', 'fontsize', label_size);
        ylabel('Angle (deg)', 'fontsize', label_size);
        title('Joint Space PID CTM Controller', 'fontsize', title_size);
        legend('Desired', 'Current');
        
        % Draw angular velocity
        FG3 = figure('Color', [1 1 1]);
        plot(save_time, save_dq_d*180/pi, ':k', 'LineWidth', t_line);
        hold on;
        plot(save_time, save_dq*180/pi, 'r', 'LineWidth', c_line);
        hold on;
        
        axis([st ft -90 90]);
        grid on;
        
        xlabel('time (s)', 'fontsize', label_size);
        ylabel('Velocity (deg/s)', 'fontsize', label_size);
        title('Joint Space PID CTM Controller', 'fontsize', title_size);
        legend('Desired', 'Current');
    end
end
