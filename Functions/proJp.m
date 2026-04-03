function Jp=proJp(Q,Qp,L)
%derivata del jacobiano rispetto al tempo

rho=Q(1);
alfa=Q(2);
beta=Q(3);

rhop=Qp(1);
alfap=Qp(2);
betap=Qp(3);

l1=L(1);
l2=L(2);
l3=L(3);
l4=L(4);
 
Jp(1,1)=0;
Jp(1,2)=0;
Jp(1,3)=-l4*cos(beta)*betap;

Jp(2,1)=0;
Jp(2,2)=-cos(alfa)*alfap*(l3+l4*sin(beta))-sin(alfa)*l4*cos(beta)*betap;  
Jp(2,3)=-sin(alfa)*alfap*l4*cos(beta)-cos(alfa)*l4*sin(beta)*betap;

Jp(3,1)=0;
Jp(3,2)=-sin(alfa)*alfap*(l3+l4*sin(beta))+cos(alfa)*l4*cos(beta)*betap;
Jp(3,3)=cos(alfa)*alfap*l4*cos(beta)-sin(alfa)*l4*sin(beta)*betap;

end