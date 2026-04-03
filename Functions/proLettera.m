function P = proLettera(m,xoff,zoff)

% Ax + By + Cz + D = 0 eqauzione del piano
% Z = m*x + offset

% CREAZIONE DEL PIANO INCLINATO

Z = @(x) m.*(x-xoff)+zoff;
z_i = Z(0);
x_f = fsolve(Z,1100);
x = linspace(400,800,101);
z = m.*(x-xoff)+zoff;

%--------------------------------------------------------------------------

% ASSEGNAZIONE DEI PUNTI DELLA B
x1=0.68;    y1=-0.100;
x2=0.6;     y2=0;
x21=0.65;   y21=0;
x22=0.7;    y22=0;
x23=0.8;    y23=0;
x3=0.845;   y3=0;
x4=0.87;    y4=0.05;
x5=0.85;    y5=0.08;
x6=0.79;    y6=0.1;
x7=0.75;    y7=0.08;
x8=0.725;   y8=0.05;
x9=0.755;   y9=0;

P1 =[x1 y1 Z(x1)]';  
P2=[x2 y2 Z(x2)]'; 
P21=[x21 y21 Z(x21)]'; 
P22=[x22 y22 Z(x22)]'; 
P23=[x23 y23 Z(x23)]'; 
P3=[x3 y3 Z(x3)]';     
P4=[x4 y4 Z(x4)]';   
P5=[x5 y5 Z(x5)]';  
P6=[x6 y6 Z(x6)]';   
P7 =[x7 y7 Z(x7)]'; 
P8 =[x8 y8 Z(x8)]'; 
P9 =[x9 y9 Z(x9)]'; 
P = [P1 P2 P21 P22 P23 P3 P4 P5 P6 P7 P8 P9];
end
