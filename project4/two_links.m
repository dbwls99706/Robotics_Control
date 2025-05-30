function dydt = two_links(t,y)

global Iz1 Iz2 L1 L2 g m1 m2 r1 r2 tq1 tq2

th1 = y(1);
dth1 = y(2);
th2 = y(3);
dth2 = y(4);

dydt = [dth1;((-tq2+g.*m2.*r2.*cos(th1+th2)+L1.*dth1.^2.*m2.*r2.*sin(th2)).*(Iz2-L2.^2.*m2+L2.*m2.*r2.*2.0+L1.*m2.*r2.*cos(th2)))./(Iz1.*Iz2-L1.^2.*L2.^2.*m2.^2-Iz2.*L1.^2.*m1-Iz1.*L2.^2.*m2+Iz2.*L1.^2.*m2+L1.^2.*L2.^2.*m1.*m2+L1.^2.*L2.*m2.^2.*r2.*2.0+Iz2.*L1.*m1.*r1.*2.0+Iz1.*L2.*m2.*r2.*2.0-L1.^2.*m2.^2.*r2.^2.*cos(th2).^2-L1.*L2.^2.*m1.*m2.*r1.*2.0-L1.^2.*L2.*m1.*m2.*r2.*2.0+L1.*L2.*m1.*m2.*r1.*r2.*4.0)+((tq1-g.*(m1.*r1.*cos(th1)+m2.*r2.*cos(th1+th2)+L1.*m2.*cos(th1))+L1.*dth2.*m2.*r2.*sin(th2).*(dth1.*2.0+dth2)).*(Iz2-L2.^2.*m2+L2.*m2.*r2.*2.0))./(Iz1.*Iz2-L1.^2.*L2.^2.*m2.^2-Iz2.*L1.^2.*m1-Iz1.*L2.^2.*m2+Iz2.*L1.^2.*m2+L1.^2.*L2.^2.*m1.*m2+L1.^2.*L2.*m2.^2.*r2.*2.0+Iz2.*L1.*m1.*r1.*2.0+Iz1.*L2.*m2.*r2.*2.0-L1.^2.*m2.^2.*r2.^2.*cos(th2).^2-L1.*L2.^2.*m1.*m2.*r1.*2.0-L1.^2.*L2.*m1.*m2.*r2.*2.0+L1.*L2.*m1.*m2.*r1.*r2.*4.0);
    dth2;-((-tq2+g.*m2.*r2.*cos(th1+th2)+L1.*dth1.^2.*m2.*r2.*sin(th2)).*(Iz1+Iz2-L1.^2.*m1+L1.^2.*m2-L2.^2.*m2+L1.*m1.*r1.*2.0+L2.*m2.*r2.*2.0+L1.*m2.*r2.*cos(th2).*2.0))./(Iz1.*Iz2-L1.^2.*L2.^2.*m2.^2-Iz2.*L1.^2.*m1-Iz1.*L2.^2.*m2+Iz2.*L1.^2.*m2+L1.^2.*L2.^2.*m1.*m2+L1.^2.*L2.*m2.^2.*r2.*2.0+Iz2.*L1.*m1.*r1.*2.0+Iz1.*L2.*m2.*r2.*2.0-L1.^2.*m2.^2.*r2.^2.*cos(th2).^2-L1.*L2.^2.*m1.*m2.*r1.*2.0-L1.^2.*L2.*m1.*m2.*r2.*2.0+L1.*L2.*m1.*m2.*r1.*r2.*4.0)-((tq1-g.*(m1.*r1.*cos(th1)+m2.*r2.*cos(th1+th2)+L1.*m2.*cos(th1))+L1.*dth2.*m2.*r2.*sin(th2).*(dth1.*2.0+dth2)).*(Iz2-L2.^2.*m2+L2.*m2.*r2.*2.0+L1.*m2.*r2.*cos(th2)))./(Iz1.*Iz2-L1.^2.*L2.^2.*m2.^2-Iz2.*L1.^2.*m1-Iz1.*L2.^2.*m2+Iz2.*L1.^2.*m2+L1.^2.*L2.^2.*m1.*m2+L1.^2.*L2.*m2.^2.*r2.*2.0+Iz2.*L1.*m1.*r1.*2.0+Iz1.*L2.*m2.*r2.*2.0-L1.^2.*m2.^2.*r2.^2.*cos(th2).^2-L1.*L2.^2.*m1.*m2.*r1.*2.0-L1.^2.*L2.*m1.*m2.*r2.*2.0+L1.*L2.*m1.*m2.*r1.*r2.*4.0)];
