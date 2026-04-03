function [S, Sp, Spp, Tq, Q_debug, Qp_debug, Qpp_debug ,power_real, Etot]=proCinematica(L,m,Q,Qp,Qpp,tt)

n=length(tt);
dt=tt(2:end)-tt(1:end-1);

%% Dati dinamica
l1=L(1); l2=L(2); l3=L(3); l4=L(4);
Hg = [0 0 0 0; 0 0 0 0; 0 0 0 -9.81; 0 0 0 0];
phi_j(:,:,4) = zeros(4,4);

%%  Cinematica diretta 4x4

for i=1:n

    rho=Q(1,i); alfa=Q(2,i); beta=Q(3,i); 

    Mabs(:,:,1)=eye(4,4);
    Wabs(:,:,1)=zeros(4,4);
    Habs(:,:,1)=zeros(4,4);

    %rototraslazione
    
    Mrel(:,:,1)=[1 0 0 L(1)+rho; 
                 0 1 0 0; 
                 0 0 1 L(2); 
                 0 0 0 1];    %M01

    Mrel(:,:,2)=[1       0             0             0; 
                 0   cos(alfa)   -sin(alfa)  L(3)*cos(alfa); 
                 0   sin(alfa)   cos(alfa)   L(3)*sin(alfa); 
                 0       0            0              1];      %M12

    Mrel(:,:,3)=[cos(beta) -sin(beta) 0  L(4)*cos(beta); 
                 sin(beta)  cos(beta) 0  L(4)*sin(beta); 
                    0           0     1       0; 
                    0           0     0       1];%M23

    Lr(:,:,1)=[0 0 0 1; 0 0 0 0; 0 0 0 0; 0 0 0 0]; %Tx
    Lr(:,:,2)=[0 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 0];%Rx
    Lr(:,:,3)=[0 -1 0 0; 1 0 0 0; 0 0 0 0; 0 0 0 0];%Rz


    for j=1:3   
  
        Mabs(:,:,j+1) = Mabs(:,:,j)*Mrel(:,:,j);
    
    % velocità
        Wrel(:,:,j) = Lr(:,:,j)*Qp(j,i);
    
    % accelerazione    
        Hrel(:,:,j) = Wrel(:,:,j)^2+Lr(:,:,j)*Qpp(j,i);

    % trasposizione a link assoluto
        L_0(:,:,j) = Mabs(:,:,j)*Lr(:,:,j)*inv(Mabs(:,:,j));
        Wrel_0(:,:,j) = Mabs(:,:,j)*Wrel(:,:,j)*inv(Mabs(:,:,j));
        Hrel_0(:,:,j) = Mabs(:,:,j)*Hrel(:,:,j)*inv(Mabs(:,:,j));

    % Calcolo velocità e accelerazioni assolute

        Habs(:,:,j+1) = Habs(:,:,j)+Hrel_0(:,:,j)+2*Wabs(:,:,j)*Wrel_0(:,:,j);

        Wabs(:,:,j+1) = Wabs(:,:,j)+Wrel_0(:,:,j);


    end

    P=Mabs(:,4,4);
    Pp=Wabs(:,:,4)*P;
    Ppp=Habs(:,:,4)*P;

    x(i)=P(1);
    y(i)=P(2);
    z(i)=P(3);

    xp(i)=Pp(1);
    yp(i)=Pp(2);
    zp(i)=Pp(3);

    xpp(i)=Ppp(1);
    ypp(i)=Ppp(2);
    zpp(i)=Ppp(3);

%% ---------------- dinamica-----------------------------------------------
    Ep(i) = 0; Ek(i) = 0;

    for j=3:-1:1
        [Ji,cm] = proPseudoInerzia(L,m,rho);
                
        M_j(:,:,1) = [1 0 0 -(l1+rho-cm(1,1));
                      0 1 0 0; 
                      0 0 1 -(l2-cm(1,3)); 
                      0 0 0 1];

        M_j(:,:,2) = [1 0 0 0; 
                      0 1 0 -(l3-cm(2,2)); 
                      0 0 1 0; 
                      0 0 0 1];

        M_j(:,:,3) = [1 0 0 -(l4-cm(3,1));
                      0 1 0 0; 
                      0 0 1 0; 
                      0 0 0 1];

        % Rispetto al S.d.r. posto a fine link (LOCALE)
        J(:,:,j) = M_j(:,:,j)*Ji(:,:,j)*transpose(M_j(:,:,j)); 
        
        % rispetto al S.d.r. universale (GLOBALE)
        J_0(:,:,j)   = Mabs(:,:,j+1)*J(:,:,j)*transpose(Mabs(:,:,j+1));
        
        phi_p(:,:,j) = skew(Hg,J_0(:,:,j));                 % calcolo forze peso
        phi_i(:,:,j) = skew(Habs(:,:,j+1),J_0(:,:,j));      % calcolo forze inerzia 
        phi_j(:,:,j) = -phi_i(:,:,j) + phi_p(:,:,j) + phi_j(:,:,j+1); % Calcolo forze sui joint
        torques(j) = -psedot(phi_j(:,:,j),L_0(:,:,j));
        
       
        Fq1(i) = torques(1); Fq2(i) = torques(2); Fq3(i) = torques(3);
        
        
        Ep(i)=Ep(i)-trace(Hg*J_0(:,:,j));   %en potenziale
        Ek(i)=Ek(i)+1/2*trace(Wabs(:,:,j+1)*J_0(:,:,j)*transpose(Wabs(:,:,j+1)));  %en cinetica
    end

%% ------------------Potenza-Motori----------------------------------------
    ww1_1(i)=-psedot(phi_j(:,:,1),Wrel_0(:,:,1));
    ww2_1(i)=-psedot(phi_j(:,:,2),Wrel_0(:,:,2));
    ww3_1(i)=-psedot(phi_j(:,:,3),Wrel_0(:,:,3));
    ww_ext_1(i)=-psedot(phi_j(:,:,4),Wabs(:,:,3));
    
    power_real(:,i) = [ww1_1(i);ww2_1(i);ww3_1(i);ww_ext_1(i)];


%% --------------Energia-Potenziale-&-Cinetica-----------------------------

    Etot(i)=Ep(i)+Ek(i);  
  
%% ---------------debug----------------------------------------------------

    Q_debug(:,i)=proInv([x(i) y(i) z(i)],L);

end

S = [x; y; z]; Sp = [xp; yp; zp]; Spp = [xpp; ypp; zpp];
Tq = [Fq1; Fq2; Fq3];
Qp_debug(1,:)= diff(Q_debug(1,:))./dt;
Qp_debug(2,:)= diff(Q_debug(2,:))./dt;
Qp_debug(3,:)= diff(Q_debug(3,:))./dt;

Qpp_debug(1,:)= diff(Qp_debug(1,:))./dt(2:end);
Qpp_debug(2,:)= diff(Qp_debug(2,:))./dt(2:end);
Qpp_debug(3,:)= diff(Qp_debug(3,:))./dt(2:end);

end