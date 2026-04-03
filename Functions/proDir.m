function S = proDir(Q,L)
%cinematica diretta

l1=L(1);
l2=L(2);
l3=L(3);
l4=L(4);

rho=Q(1);
alfa=Q(2);
beta=Q(3);

x=rho+l1+l4*cos(beta);
y=(l3+l4*sin(beta))*cos(alfa);
z=l2+(l3+l4*sin(beta))*sin(alfa);

S=[x y z]';
end