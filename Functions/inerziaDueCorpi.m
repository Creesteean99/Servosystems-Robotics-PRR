function [J, cm] = inerziaDueCorpi(m1, m2, cm1, cm2, L,J1, J2)
l1 = L(1); l2 = L(2);
cm = (m1*cm1 + m2*cm2)/(m1+m2);

dx = cm(1)-cm1(1);
dy = cm(2)-cm1(2);
dz = cm(3)-cm1(3);

D = [dy^2+dz^2  -dx*dy  -dx*dz;
     -dx*dy dx^2+dz^2   -dy*dz;
     -dx*dz -dy*dz  dx^2+dy^2];

J1n = J1 + m1 * D; % Inerzia primo corpo rispetto al centro di massa del sistema

dx = cm(1)-0; 
dy = cm(2)-0;
dz = cm(3)-l2;

D = [dy^2+dz^2  -dx*dy  -dx*dz;
     -dx*dy dx^2+dz^2   -dy*dz;
     -dx*dz -dy*dz  dx^2+dy^2];

J2n = J2 + m2 * D; % Inerzia secondo corpo rispetto al centro di massa del sistema

J = J1n+J2n; % Inerzia complessiva del sistema nel centro di massa del sistema stesso.

end