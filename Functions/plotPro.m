function plotPro(Q,L,colore)

%figure % fig figure number

rho = Q(1);
alfa = Q(2);
beta = Q(3);

l1=L(1);
l2 = L(2);
l3 = L(3);
l4=L(4);

ll = l1+l2+l3+l4;
axis([0 ll*1.1 -ll*1.1 ll*1.1 -ll*1.1/2 ll*1.1/2])

x0 = 0;
y0 = 0;
z0 = 0;

x1 = l1;
y1 = 0;
z1 = 0;

x2 = rho+l1;
y2 = 0;
z2 = 0;

x3 = rho+l1;
y3 = 0;
z3 = l2;

%prendo alfa partendo dall'asse Y
%prendo beta partendo dall'asse X
x4 = rho+l1;
y4 = l3*cos(alfa);
z4 = l2+l3*sin(alfa);

x5 = rho+l1+l4*cos(beta);        %gripper
y5 = (l3+l4*sin(beta))*cos(alfa);
z5 = l2+(l3+l4*sin(beta))*sin(alfa);

hold on
plot3([x0 x1 x2 x3 x4 x5],[y0 y1 y2 y3 y4 y5],[z0 z1 z2 z3 z4 z5], 'LineWidth',2,'color',colore); %manipulator
plot3(0,0,0,'*','color','k'); %base
plot3([x3 x4],[y3 y4],[z3 z4], 'o','color',colore); %polar joint position
plot3([x1 x2],[y1 y2],[z1 z2], 's','color',colore); %prismatic joint position
grid on
xlabel('x')
ylabel('y')
zlabel('z')
end