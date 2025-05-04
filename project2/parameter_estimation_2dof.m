clear all
close all

global I1 I2 Im1 Im2 L1 L2 m1 m2 r1 r2 g Fs1 Fs2 Fv1 Fv2 tau1 tau2

I1 = 0.05;	I2 = 0.05;          % 링크 관성
Im1 = 0.05;	Im2 = 0.05;         
L1 = 0.5;	L2 = 0.5;           % 링크 길이
m1 = 0.2;	m2 = 0.2;           % 막대기 질량
r1 = 0.1;	r2 = 0.1;           % 무게중심
g = 9.806;                          % 중력가속도

Fs1 = 0.1;	
Fs2 = 0.1;          

Fv1 = 0.1;	
Fv2 = 0.1;          

q1 = pi/4;           % rad 
dq1 = 0;             % rad/s

q2 = pi/4;           % rad   
dq2=0;               % rad/s

W1_int = [0 0 0 0 0 0 0 0 0 0;
          0 0 0 0 0 0 0 0 0 0];
theta = [0; 0; 0; 0; 0;
         0; 0; 0; 0; 0];
theta2 = [0; 0; 0; 0; 0;
         0; 0; 0; 0; 0];
P = eye(10);                    % 단위행렬 생성
u = [0; 0];

data = [];                      
data2 = [];                     
time = [];                      % 스택 변수들


dt = 0.005; ft = 5;             %샘플링 타임
n = 1;

temp = zeros(10,10);            % 임시 error 알고리즘 행렬
temp2 = zeros(10,1);            

% 시뮬레이션
for cnt=0:dt:ft
    tau1 = sin(cnt) + cos(10*cnt);
    tau2 = sin(cnt) + cos(10*cnt);
    
    % Return the dynamics of each link
    [t, y] = ode45('two_link', [0 dt], [q1; dq1; q2; dq2]);
    index = length(y);
    
    % Update dynamics of each link
    q1 = y(index, 1);
    dq1 = y(index, 2);
    q2 = y(index, 3);
    dq2 = y(index, 4);
    
    % W1 integration
    W1_int = W1_int + [0 0 0                      g*cos(q1) g*cos(q1+q2) 0 -sign(dq1) 0          -dq1 0; 
                       0 0 -dq1*(dq1+dq2)*sin(q2) 0         g*cos(q1+q2) 0 0          -sign(dq2) 0    -dq2]*dt;
    W2 = [dq1 dq1+dq2 (2*dq1+dq2)*cos(q2) 0 0 0   0 0 0 0;
          0   dq1+dq2 dq1*cos(q2)         0 0 dq2 0 0 0 0];
    Y = W2 - W1_int;                            
    u = u + [tau1; tau2]*dt;                    
    
    % Kalman Filter based parameter estimation algorithm
    P = P - P*Y.'*inv(eye(2) + Y*P*Y.')*Y*P;    
    K = P*Y.';                                  
    theta = theta + K*(u - Y*theta);            
    data(n,:) = theta;                          
    
    % error 최소화 알고리즘
    temp = temp + Y.'*Y;                        
    temp2 = temp2 + Y.'*u;                      
    theta2 = inv(temp)*temp2;                   
    data2(n,:) = theta2;                        
    
    time(n,:) = cnt;
    n = n + 1;
    
    % 계산 시간 표시
    cmd = sprintf("Time: %2.2f", cnt);
    clc
    disp(cmd);
end

% 그래프 출력
FG = figure('Color', [1 1 1]);
AX = axes('parent', FG);

plot(time(:,1), data(:,1), 'k', 'linew', 2);
grid on;
hold on;
plot(time(:,1), data(:,2), 'r', 'linew', 2);
plot(time(:,1), data(:,3), 'g', 'linew', 2);
plot(time(:,1), data(:,4), 'b', 'linew', 2);
plot(time(:,1), data(:,5), 'y', 'linew', 2);
plot(time(:,1), data(:,6), '--k', 'linew', 2);
plot(time(:,1), data(:,7), 'c', 'linew', 2);
plot(time(:,1), data(:,8), '--c', 'linew', 2);
plot(time(:,1), data(:,9), 'm', 'linew', 2);
plot(time(:,1), data(:,10), '--m', 'linew', 2);

xlabel('time', 'fontsize', 15);
ylabel('parameter', 'fontsize', 15);
title('Kalman Filter based estimation algorithm', 'fontsize', 20);
legend({'I1+m2L1^2+Im1=0.15', 'I2=0.05', 'm2r2L1=0.01', 'm1r1+m2L1=0.12', 'm2r2=0.02', ...
        'Im2=0.05', 'Fs1=0.1', 'Fs2=0.1', 'Fv1=0.1', 'Fv2=0.1'}, ...
        'Location', 'best');

%% Plot result of Error minimization algorithm
FG2 = figure('Color', [1 1 1]);
AX2 = axes('parent', FG2);

plot(time(:,1), data2(:,1), 'k', 'linew', 2);
grid on;
hold on;
plot(time(:,1), data2(:,2), 'r', 'linew', 2);
plot(time(:,1), data2(:,3), 'g', 'linew', 2);
plot(time(:,1), data2(:,4), 'b', 'linew', 2);
plot(time(:,1), data2(:,5), 'y', 'linew', 2);
plot(time(:,1), data2(:,6), '--k', 'linew', 2);
plot(time(:,1), data2(:,7), 'c', 'linew', 2);
plot(time(:,1), data2(:,8), '--c', 'linew', 2);
plot(time(:,1), data2(:,9), 'm', 'linew', 2);
plot(time(:,1), data2(:,10), '--m', 'linew', 2);

xlabel('time', 'fontsize', 15);
ylabel('parameter', 'fontsize', 15);
title('Error minimization algorithm', 'fontsize', 20);
legend({'I1+m2L1^2+Im1=0.15', 'I2=0.05', 'm2r2L1=0.01', 'm1r1+m2L1=0.12', 'm2r2=0.02', ...
        'Im2=0.05', 'Fs1=0.1', 'Fs2=0.1', 'Fv1=0.1', 'Fv2=0.1'}, ...
        'Location', 'best');