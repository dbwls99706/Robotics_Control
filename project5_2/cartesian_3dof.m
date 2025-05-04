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
m1 = 0.2;   m2 = 0.2;   m3 = 0.2;            %막대기 질량
L1 = 0.5;   L2 = 0.5;   L3 = 0.5;            %링크 길이
r1 = 0.1;   r2 = 0.1;   r3 = 0.1;            %질량 중심
Iz1 = 0.05; Iz2 = 0.05; Iz3 = 0.05;          %링크 관성

%계산용 변수 초기화
init_q1 = -pi/2; init_q2 = pi/2; init_q3 = -pi/2;      
init_dq1 = 0.00; init_dq2 = 0.00; init_dq3 = 0.00;      
q = [init_q1; init_q2; init_q3];                        
dq = [init_dq1; init_dq2; init_dq3];                      

init_X = three_link_K(q(1), q(2), q(3));    
X = init_X;                                             % 현재 위치
dX = [0; 0; 0];                                         % 현재 속도
X_d = init_X;                                           % 타겟 위치
dX_d = [0; 0; 0];                                       % 타겟 속도
ddX_d = [0; 0; 0];                                      % 타겟 가속도

% 적분기용 변수
X_err_sum = 0;              

%토크용 변수
tau1 = 0.0;   
tau2 = 0.0;    
tau3 = 0.0;                 
tau = [tau1; tau2; tau3];   

% 제어기 수치
Wn = 30;                % 제어주기
Kp = Wn^2;              % P 게인
Kv = 2*Wn;              % D 게인
Ki = 150;               % I 게인


if (flag_sim == 1)
    

    n = 1;
    sin_t = 0;
    pre_J = 0;

    for time = st:dt:ft

        if (time < 1.0)
            X_d = init_X;
            dX_d = [0; 0];
            ddX_d = [0; 0];
        elseif (time < 2.0)
            X_d(1) = init_X(1);
            if (X_d(2) < init_X(2)+0.1)             % 타겟 위치
                X_d(2) = X_d(2) + (0.1/0.5)*dt;     % 타겟 속도
            else
                X_d(2) = init_X(2) + 0.1;
            end
            dX_d = (X_d - [save_X_x_d(n-1); save_X_y_d(n-1)])./dt;
            ddX_d = (dX_d - [save_dX_x_d(n-1); save_dX_y_d(n-1)])./dt;
        else
            X_d = [0.1*sin((2*pi*sin_t)) + init_X(1);
                   0.1*cos((2*pi*sin_t)) + init_X(2)];
            sin_t = sin_t + dt;
            dX_d = (X_d - [save_X_x_d(n-1); save_X_y_d(n-1)])./dt;
            ddX_d = (dX_d - [save_dX_x_d(n-1); save_dX_y_d(n-1)])./dt;
        end
        
        % 기구학 적용
        J = three_link_J(q(1), q(2), q(3));	% 야코비안 계산
        dJ = (J - pre_J)/dt;                            
        pre_J = J;
        
        X = three_link_K(q(1), q(2), q(3));	
        dX = J*dq;                                      
        D = three_link_I(q(1), q(2), q(3));                        % 관성계산
        H = three_link_C(q(1), q(2), q(3), dq(1), dq(2), dq(3));   % H텀 계산
        G = three_link_G(q(1), q(2), q(3));                        % 중력계산
        
        % 제어기
        X_err_sum = X_err_sum + (X_d-X)*dt;                         % 적분기
        u = ddX_d + Kv*(dX_d - dX) + Kp*(X_d - X) + Ki*X_err_sum;	% 제어기
        ddq_ref = J\(u - dJ*dq);                                    
        gravity_err = 1.2;                                          % 중력오류보상
        tq_ctrl = D*ddq_ref + H + G*gravity_err;                    % 토크제어
        
        % 역기구학
        tau = tq_ctrl;
        tau1 = tau(1);
        tau2 = tau(2);
        tau3 = tau(3);
        
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
        save_X_x(n) = X(1);          % [m]
        save_X_y(n) = X(2);          % [m]
        save_dX_x(n) = dX(1);        % [m/s]
        save_dX_y(n) = dX(2);        % [m/s]
        save_X_x_d(n) = X_d(1);      % [m], target value
        save_X_y_d(n) = X_d(2);      % [m], target value
        save_dX_x_d(n) = dX_d(1);    % [m/s], target value
        save_dX_y_d(n) = dX_d(2);    % [m/s], target value
        n = n + 1;
    end
end

if (flag_draw == 1)
    font_size_label = 20;
    font_size_title = 25;
    linewidth_current = 3;
    linewidth_target = 5;
    
    if (flag_draw_robot == 1)
        % save as gif
        filename = 'cartesian_3dof.gif';
        
        % Draw robot
        x1 = L1*cos(init_q1);               % 1번 조인트 x 축 위치
        y1 = L1*sin(init_q1);               % 1번 조인트 y 축 위치
        x2 = L2*cos(init_q1 + init_q2);     % 2번 조인트 x 축 위치
        y2 = L2*sin(init_q1 + init_q2);     % 2번 조인트 y 축 위치
        x3 = L3*cos(init_q1 + init_q2 + init_q3);     % 3번 조인트 x 축 위치
        y3 = L3*sin(init_q1 + init_q2 + init_q3);     % 3번 조인트 y 축 위치
        
        FG1 = figure('Color', [1 1 1]);
        AX = axes('parent', FG1);
        hold on;
        
        Px1 = [0 x1];               Py1 = [0 y1];
        Px2 = [x1 x1+x2];           Py2 = [y1 y1+y2];
        Px3 = [x1+x2 x1+x2+x3];     Py3 = [y1+y2 y1+y2+y3];
        
        p1 = plot(Px1, Py1, '-ob', 'Linewidth', linewidth_current);
        p2 = plot(Px2, Py2, '-or', 'Linewidth', linewidth_current);
        p3 = plot(Px3, Py3, '-og', 'Linewidth', linewidth_current);
        
        axis([-0.6 1.0 -1.4 0.4]);
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
            
            q1 = save_q1(n);             % [deg], joint 1 angle
            q2 = save_q2(n);             % [deg], joint 2 angle
            q3 = save_q3(n);             % [deg], joint 3 angle
            x1 = L1*cos(q1);            % [m], joint 1 X-axis position
            y1 = L1*sin(q1);            % [m], joint 1 Y-axis position
            x2 = L2*cos(q1+q2);         % [m], joint 2 X-axis position
            y2 = L2*sin(q1+q2);         % [m], joint 2 Y-axis position
            x3 = L3*cos(q1+q2+q3);      % [m], joint 3 X-axis position
            y3 = L3*sin(q1+q2+q3);      % [m], joint 3 Y-axis position
            
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
        % Draw position
        FG2 = figure('Color', [1 1 1]);
        plot(save_time, save_X_x_d, ':r', 'LineWidth', linewidth_target);
        hold on;
        plot(save_time, save_X_y_d, ':b', 'LineWidth', linewidth_target);
        hold on;
        
        plot(save_time, save_X_x, 'r', 'LineWidth', linewidth_current);
        hold on;
        plot(save_time, save_X_y, 'b', 'LineWidth', linewidth_current);
        hold on;
        
        axis([st ft -1.25 1]);
        grid on;
        
        xlabel('time (s)', 'fontsize', font_size_label);
        ylabel('Position (m)', 'fontsize', font_size_label);
        title('Cartesian Space PID CTM Controller', 'fontsize', font_size_title);
        legend({'tar_x', 'tar_y', 'cur_x', 'cur_y'}, 'location', 'best', 'orientation', 'horizontal', 'fontsize', 15);
        
        % Draw velocity
        FG3 = figure('Color', [1 1 1]);
        plot(save_time, save_dX_x_d, ':r', 'LineWidth', linewidth_target);
        hold on;
        plot(save_time, save_dX_y_d, ':b', 'LineWidth', linewidth_target);
        hold on;
        
        plot(save_time, save_dX_x, 'r', 'LineWidth', linewidth_current);
        hold on;
        plot(save_time, save_dX_y, 'b', 'LineWidth', linewidth_current);
        hold on;
        
        axis([st ft -1.25 1.25]);
        grid on;
        
        xlabel('time (s)', 'fontsize', font_size_label);
        ylabel('Velocity (m/s)', 'fontsize', font_size_label);
        title('Cartesian Space PID CTM Controller', 'fontsize', font_size_title);
        legend({'tar_{dx}', 'tar_{dy}', 'cur_{dx}', 'cur_{dy}'}, 'location', 'best', 'orientation', 'horizontal', 'fontsize', 15);
    end
end
