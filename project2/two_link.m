function dydt = two_link(t, y)

global I1 I2 Im1 Im2 L1 L2 m1 m2 r1 r2 g Fs1 Fs2 Fv1 Fv2 tau1 tau2
q1 = y(1);  
dq1 = y(2);

q2 = y(3);  
dq2 = y(4);

M = [(I1+m2*(L1^2)+Im1)+I2+2*(m2*r2*L1)*cos(q2) I2+(m2*r2*L1)*cos(q2);
     I2+(m2*r2*L1)*cos(q2)            I2+Im2];
C = [-m2*r2*L1*sin(q2)*dq2 -m2*r2*L1*sin(q2)*(dq1+dq2);
     m2*r2*L1*sin(q2)*dq1  0];
G = [-m1*r1*g*cos(q1)-m2*L1*g*cos(q1)-m2*r2*g*cos(q1+q2);
     -m2*r2*g*cos(q1+q2)];
d = [Fs1*sign(dq1)+Fv1*dq1;
     Fs2*sign(dq2)+Fv2*dq2];

ddq = inv(M)*([tau1; tau2] - C*[dq1; dq2] - G - d);

dydt = [dq1; ddq(1); dq2; ddq(2)];
