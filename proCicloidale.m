function [Q, Qp, Qpp, tt]=proCicloidale(L,V,A,Si,Sf)

    d=[sqrt((Sf(1)-Si(1))^2) sqrt((Sf(2)-Si(2))^2) sqrt((Sf(3)-Si(3))^2)];  %distanza tra i punti    
    
    T=max([max([(2*d(1))/V(1) sqrt((2*pi*d(1))/A(1))]) max([(2*d(2))/V(2) sqrt((2*pi*d(2))/A(2))]) max([(2*d(3))/V(3) sqrt((2*pi*d(3))/A(3))])]);
    
    tt=linspace(0,T,1001);

    Qi=proInv(Si,L);
    Qf=proInv(Sf,L);

    n=length(tt);

    for i=1:n
        [rho,rhop, rhopp] = cicloidale(tt(i),T,Qi(1), Qf(1)-Qi(1));
        [alfa,alfap, alfapp] = cicloidale(tt(i),T,Qi(2), Qf(2)-Qi(2));
        [beta,betap, betapp] = cicloidale(tt(i),T,Qi(3), Qf(3)-Qi(3));
        
        Q(:,i)=[rho,alfa,beta]';
        Qp(:,i)=[rhop,alfap,betap]';
        Qpp(:,i)=[rhopp,alfapp,betapp]';
    end

end