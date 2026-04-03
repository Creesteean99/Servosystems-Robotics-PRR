function Qn = proInv(S,L)
x = S(1);
y = S(2);
z = S(3);

l1 = L(1);
l2 = L(2);
l3 = L(3);
l4 = L(4);

alfa = atan2((z-l2),y);
beta = asin(((z-l2)/sin(alfa)-l3)/l4);
rho = x-l1-l4*cos(beta);

Qn=[rho alfa beta]';

end
