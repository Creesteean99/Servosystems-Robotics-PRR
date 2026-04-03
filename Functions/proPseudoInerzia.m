function [J,cm] = proPseudoInerzia(L, m, rho)

% clear, clc
l1 = L(1); l2 = L(2); l3 = L(3); l4 = L(4); 
m1 = m(1); m2 = m(2); m3 = m(3); m4 = m(4);

% Funzione richiede: Vettore L, R, m; Rho

%% Link 1

% Parte orizzontale
% li faccio rispetto centro di massa
J1 = [0,          0,          0;
      0,      m1*l1^2/12,      0;
      0,          0,      m1*l1^2/12]; 


cm1 = [l1/2+rho 0 0];

% Parte verticale
% li faccio rispetto centro di massa
J2 = [m2*l2^2/12,     0,            0;
      0,             m2*l2^2/12,    0;
      0,             0,            0];

cm2 = [l1+rho 0 l2/2];


%% Link 1 Totale

[J_tilde(:,:,1), cm(1,:)] = inerziaDueCorpi(m1,m2,cm1,cm2,L,J1,J2);
%% Link 2

J_tilde(:,:,2) = [m3*l3^2/12,      0,            0;
                  0,               0,            0;
                  0,               0,         m3*l3^2/12]; 

cm(2,:) = [0 l3/2 0];
%% Link 3 

J_tilde(:,:,3) = [0,          0,          0;
                  0,      m4*l4^2/12,      0;
                  0,          0,      m4*l4^2/12]; 


cm(3,:) = [l4/2 0 0];
%% Pseudo Inerzia

M = [m1+m2 m3 m4];
 
for j = 1:3

    % Rispetto al S.d.r. baricentrico

    % Conversione diagonale principale
    J(1,1,j) = (-J_tilde(1,1,j)+J_tilde(2,2,j)+J_tilde(3,3,j))/2;
    J(2,2,j) = (-J_tilde(2,2,j)+J_tilde(1,1,j)+J_tilde(3,3,j))/2;
    J(3,3,j) = (-J_tilde(3,3,j)+J_tilde(2,2,j)+J_tilde(1,1,j))/2;
    J(4,4,j) = M(j);
    
    % xy
    J(1,2,j) = -J_tilde(1,2,j);
    J(2,1,j) = J(1,2,j);
    
    % yz
    J(2,3,j) = -J_tilde(2,3,j);
    J(3,2,j) = J(2,3,j);
    
    % xz
    J(1,3,j) = -J_tilde(1,3,j);
    J(3,1,j) = J(1,3,j);
    
    J(1,4,j) = M(j)*0; J(4,1,j) = J(1,4,j);
    J(2,4,j) = M(j)*0; J(4,2,j) = J(2,4,j);
    J(3,4,j) = M(j)*0; J(4,3,j) = J(3,4,j);

end


end
