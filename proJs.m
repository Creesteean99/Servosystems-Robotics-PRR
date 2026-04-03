function Js = proJs()

syms aa bb rho l1 l2 l3 l4

x=rho+l1+l4*cos(bb);
y=(l3+l4*sin(bb))*cos(aa);
z=l2+(l3+l4*sin(bb))*sin(aa);

Js = [diff(x,aa) diff(x,bb) diff(x,rho);
   diff(y,aa) diff(y,bb) diff(y,rho);
   diff(z,aa) diff(z,bb) diff(z,rho)];