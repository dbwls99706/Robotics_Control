clc
clear all
close all
%% 3-DOF Simulation
global L1 L2 L3 m1 m2 m3 r1 r2 r3 Iz1 Iz2 Iz3 g tau1 tau2 tau3
L1 = 0.5; L2 = 0.5; L3 = 0.5;          % 링크 길이
r1 = 0.1; r2 = 0.1; r3 = 0.1;          % 링크 중심
m1 = 0.2; m2 = 0.2; m3 = 0.2;          % 링크 무게
Iz1 = 0.05; Iz2 = 0.05; Iz3 = 0.05;    % 링크 관성

g = 9.806;                             % 중력가속도

% 초기 자세
q1 = -pi; dq1 = 0;
q2 = -pi/4;  dq2 = 0;
q3 = -pi/4; dq3 = 0;

%%
Fi = figure('Color', [1 1 1]);
Ax = axes('parent', Fi);
hold on;
grid on;
axis([-1.5 1.5 -1.5 1.5]);

x1 = L1*cos(q1);             % 첫 번째 링크 좌표
y1 = L1*sin(q1);
Px1 = [0, x1];
Py1 = [0, y1];

x2 = L2*cos(q1+q2);          % 두 번째 링크 좌표
y2 = L2*sin(q1+q2);
Px2 = [x1, x1+x2];
Py2 = [y1, y1+y2];

x3 = L3*cos(q1+q2+q3);       % 세 번째 링크 좌표
y3 = L3*sin(q1+q2+q3);
Px3 = [x1+x2, x1+x2+x3];
Py3 = [y1+y2, y1+y2+y3];

p1 = plot(Px1, Py1, '-b', 'Linewidth', 3);
p2 = plot(Px2, Py2, '-r', 'Linewidth', 3);
p3 = plot(Px3, Py3, '-g', 'Linewidth', 3);

xlabel('X-axis (m)', 'fontsize', 20);
ylabel('Y-axis (m)', 'fontsize', 20);
title('FreeFall 3DOF Simulation', 'fontsize', 25);

%% ode45
% 시간
dt = 0.010; ft = 5;
n = 1;

for cnt=0:dt:ft
    % 자유낙하 시스템
    % 각 링크의 토크=0
    tau1 = 0;              % 각 링크의 토크
    tau2 = 0;
    tau3 = 0;
    
    % 각 링크의 역학 관계
    [t,y] = ode45('three_link', [0 dt], [q1; dq1; q2; dq2; q3; dq3]);
    index = length(y);
    
    % 각 링크의 역학 갱신
    q1 = y(index, 1);
    dq1 = y(index, 2);
    q2 = y(index, 3);
    dq2 = y(index, 4);
    q3 = y(index, 5);
    dq3 = y(index, 6);
    
    % 첫 번째 링크 끝점
    x1 = L1*cos(q1);
    y1 = L1*sin(q1);
    Px1 = [0, x1];
    Py1 = [0, y1];
    
    % 두 번째 링크 끝점
    x2 = L2*cos(q1+q2);
    y2 = L2*sin(q1+q2);
    Px2 = [x1, x1+x2];
    Py2 = [y1, y1+y2];
    
    % 세 번째 링크 끝점
    x3 = L3*cos(q1+q2+q3);
    y3 = L3*sin(q1+q2+q3);
    Px3 = [x1+x2, x1+x2+x3];
    Py3 = [y1+y2, y1+y2+y3];
    
    % 현재 상태 설정 및 플롯
    n = n + 1;
    set(p1, 'XData', Px1, 'YData', Py1);
    set(p2, 'XData', Px2, 'YData', Py2);
    set(p3, 'XData', Px3, 'YData', Py3);
    drawnow


end
