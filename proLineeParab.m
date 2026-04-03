function [Theta, Thetap, Thetapp, Tt] = proLineeParab(Qp_max, Qpp_const, Qpp_max, Punti)

coeff = 1;    % Questo coefficiente verrà impiegato per aumentare i tau qualora l'algoritmo rilevasse che le velocità e le accelerazioni superano quella limite

error = 0; % flag errore

figure(80), figure(81), figure(82)
clf(80), clf(81), clf(82);

% Calcolo del tempo di raccordo
n = length(Punti);

for i = 1:n
    Theta_j(:,i) = Punti(:,i);
end

while 1  
 while 1

    % VETTORE DEI TAU 
    for i=1:n-1
        Tauj_a   = (abs(Theta_j(1,i+1)-Theta_j(1,i))/Qp_max(1))*coeff; % (giunto motore alpha) 
        Tauj_b   = (abs(Theta_j(2,i+1)-Theta_j(2,i))/Qp_max(2))*coeff; % (giunto motore beta)
        Tauj_rho = (abs(Theta_j(3,i+1)-Theta_j(3,i))/Qp_max(3))*coeff; % (giunto motore rho)
        maxTau = max([Tauj_a,Tauj_b,Tauj_rho]);
        Tau_j(i)= maxTau;
        Tj(1) = 0; Tj(i+1) = Tj(i)+maxTau;
    end
    
    % Punto iniziale alpha
    Thetap_a(1)   = (Theta_j(1,2)-Theta_j(1,1))/Tau_j(1);
    Thetapp_a(1)  = sign(Thetap_a(1))*Qpp_max(1);
    tj_a(1) = Tau_j(1)-sqrt(Tau_j(1)^2-2*(Theta_j(1,2)-Theta_j(1,1))/Thetapp_a(1));  
    
    % Punto iniziale beta
    Thetap_b(1)   = (Theta_j(2,2)-Theta_j(2,1))/Tau_j(1); 
    Thetapp_b(1)  = sign(Thetap_b(1))*Qpp_max(2);
    tj_b(1) = Tau_j(1)-sqrt(Tau_j(1)^2-2*(Theta_j(2,2)-Theta_j(2,1))/Thetapp_b(1)); 
    
    
    % Punto iniziale rho
    Thetap_r(1)   = (Theta_j(3,2)-Theta_j(3,1))/Tau_j(1);
    Thetapp_r(1)  = sign(Thetap_r(1))*Qpp_max(3);
    tj_r(1) = Tau_j(1)-sqrt(Tau_j(1)^2-2*(Theta_j(3,2)-Theta_j(3,1))/Thetapp_r(1));
    
    
    % VETTORE DELLE VELOCITA' 
    for i=2:n-1
        Thetap_a(i)   = (Theta_j(1,i+1)-Theta_j(1,i))/Tau_j(i); % (giunto motore alpha)
        Thetap_b(i)   = (Theta_j(2,i+1)-Theta_j(2,i))/Tau_j(i); % (giunto motore beta)
        Thetap_r(i)   = (Theta_j(3,i+1)-Theta_j(3,i))/Tau_j(i); % (giunto motore rho)
    end
    
    Thetap_a(n) = 0;
    Thetap_b(n) = 0;
    Thetap_r(n) = 0;
    
    for i= 1:n
        % VETTORE DELLE ACCELERAZIONI
        if i == 1
            Thetapp_a(i) = sign(Thetap_a(i))*Qpp_max(1);
            Thetapp_b(i) = sign(Thetap_b(i))*Qpp_max(2);
            Thetapp_r(i) = sign(Thetap_r(i))*Qpp_max(3);
        else
            Thetapp_a(i) = sign(Thetap_a(i)-Thetap_a(i-1))*Qpp_max(1);
            Thetapp_b(i) = sign(Thetap_b(i)-Thetap_b(i-1))*Qpp_max(2);
            Thetapp_r(i) = sign(Thetap_r(i)-Thetap_r(i-1))*Qpp_max(3);
        end
    end
    % VETTORE DEI TEMPI tj
    
    % Punti intermedi
    for i=2:n-1
        tj_a(i) = abs(Thetap_a(i)-Thetap_a(i-1))/Qpp_max(1);
        tj_b(i) = abs(Thetap_b(i)-Thetap_b(i-1))/Qpp_max(2);
        tj_r(i) = abs(Thetap_r(i)-Thetap_r(i-1))/Qpp_max(3);
    end
    
    % Punto finale
    tj_a(n) = Tau_j(n-1)-sqrt(Tau_j(n-1)^2-2*(Theta_j(1,n)-Theta_j(1,n-1))/(-Thetapp_a(n)));
    tj_b(n) = Tau_j(n-1)-sqrt(Tau_j(n-1)^2-2*(Theta_j(2,n)-Theta_j(2,n-1))/(-Thetapp_b(n)));
    tj_r(n) = Tau_j(n-1)-sqrt(Tau_j(n-1)^2-2*(Theta_j(3,n)-Theta_j(3,n-1))/(-Thetapp_r(n)));
    
    % Vettore per nuova traiettoria con punti fittizzi
    Theta_a_ext = [Theta_j(1,1), Theta_j(1,1), Theta_j(1,2),Theta_j(1,3), Theta_j(1,4), Theta_j(1,5), Theta_j(1,6), Theta_j(1,7), Theta_j(1,8), Theta_j(1,9), Theta_j(1,10), Theta_j(1,11), Theta_j(1,12), Theta_j(1,12)];
    Theta_b_ext = [Theta_j(2,1), Theta_j(2,1), Theta_j(2,2),Theta_j(2,3), Theta_j(2,4), Theta_j(2,5), Theta_j(2,6), Theta_j(2,7), Theta_j(2,8), Theta_j(2,9), Theta_j(2,10), Theta_j(2,11), Theta_j(2,12), Theta_j(2,12)];
    Theta_r_ext = [Theta_j(3,1), Theta_j(3,1), Theta_j(3,2),Theta_j(3,3), Theta_j(3,4), Theta_j(3,5), Theta_j(3,6), Theta_j(3,7), Theta_j(3,8), Theta_j(3,9), Theta_j(3,10), Theta_j(3,11), Theta_j(3,12), Theta_j(3,12)];
    
    % Calcolo nuove velocità e accelerazioni
    
    for i=1:n
        tj(i) = [max([tj_a(i) tj_b(i) tj_r(i)])];
        if tj(i) < 1e-6
            tj(i) = 0;
        end
    end  
    
    Thetap_a(1) =  (Theta_j(1,2)-Theta_j(1,1))/(Tau_j(1)-(tj(1)/2));
    Thetap_b(1) =  (Theta_j(2,2)-Theta_j(2,1))/(Tau_j(1)-(tj(1)/2));
    Thetap_r(1) =  (Theta_j(3,2)-Theta_j(3,1))/(Tau_j(1)-(tj(1)/2));
    Thetap_a(n-1) = (Theta_j(1,n)-Theta_j(1,n-1))/(Tau_j(n-1)-(tj(n)/2));
    Thetap_b(n-1) = (Theta_j(2,n)-Theta_j(2,n-1))/(Tau_j(n-1)-(tj(n)/2));
    Thetap_r(n-1) = (Theta_j(3,n)-Theta_j(3,n-1))/(Tau_j(n-1)-(tj(n)/2));
    
    for i=1:n
        if tj(i) == 0
            Thetapp_a(i) = 0;
            Thetapp_b(i) = 0;
            Thetapp_r(i) = 0;
        elseif i == 1
            Thetapp_a(i) = Thetap_a(i)/tj(i);
            Thetapp_b(i) = Thetap_b(i)/tj(i);
            Thetapp_r(i) = Thetap_r(i)/tj(i);
        else
            Thetapp_a(i) = (Thetap_a(i)-Thetap_a(i-1))/tj(i);
            Thetapp_b(i) = (Thetap_b(i)-Thetap_b(i-1))/tj(i);
            Thetapp_r(i) = (Thetap_r(i)-Thetap_r(i-1))/tj(i);
        end
    end

    Thetap_max = [max(abs(Thetap_a)), max(abs(Thetap_b)), max(abs(Thetap_r))];
    Thetapp_max = [max(abs(Thetapp_a)), max(abs(Thetapp_b)), max(abs(Thetapp_r))];

    Tj_ext = [Tj(1)+tj(1)/2, Tj(2), Tj(3), Tj(4), Tj(5), Tj(6), Tj(7), Tj(8), Tj(9), Tj(10), Tj(11), Tj(12) - tj(n)/2];

    for i = 1:length(Tj)-1
        if Tj_ext(i) + tj(i)/2 > Tj_ext(i+1) - tj(i+1)/2
            error = 1;
        end
    end

    if error ~= 1 && Thetap_max(1) < Qp_max(1) && Thetap_max(2) < Qp_max(2) && Thetap_max(3) < Qp_max(3) && Thetapp_max(1) < Qpp_const(1) && Thetapp_max(2) < Qpp_const(2) && Thetapp_max(3) < Qpp_const(3) && isreal(tj)
        break;
    else
        coeff = coeff+0.5;
        error = 0;
    end

end %while
    
    Tj_ext = [Tj(1), Tj(1)+tj(1)/2, Tj(2), Tj(3), Tj(4), Tj(5), Tj(6), Tj(7), Tj(8), Tj(9), Tj(10), Tj(11), Tj(12)-tj(n)/2, Tj(12)];
    Parab_Periods = [Tj(1)+tj(1), Tj(2)-tj(2)/2, Tj(2)+tj(2)/2, Tj(3)-tj(3)/2, Tj(3)+tj(3)/2, Tj(4)-tj(4)/2, Tj(4)+tj(4)/2, Tj(5)-tj(5)/2, Tj(5)+tj(5)/2, Tj(6)-tj(6)/2, Tj(6)+tj(6)/2, Tj(7)-tj(7)/2, Tj(7)+tj(7)/2, Tj(8)-tj(8)/2, Tj(8)+tj(8)/2, Tj(9)-tj(9)/2, Tj(9)+tj(9)/2, Tj(10)-tj(10)/2, Tj(10)+tj(10)/2, Tj(11)-tj(11)/2, Tj(11)+tj(11)/2, Tj(12)-tj(12)];
    
    % LEGGE DI MOTO ALPHA
    figure(80), hold on,
    subplot(3,1,1), hold on,
    plot(Tj_ext(:),Theta_a_ext(:),'*--k'), title("Traiettoria (X)"), hold on
    xline(Parab_Periods(:), '--k')
    subplot(3,1,2), hold on
    xline(Parab_Periods(:), '--k')
    subplot(3,1,3), hold on
    xline(Parab_Periods(:), '--k')

    figure(81), hold on,
    subplot(3,1,1), hold on,
    plot(Tj_ext(:),Theta_b_ext(:),'*--k'), title("Traiettoria (Y)"), hold on
    xline(Parab_Periods(:), '--k')
    subplot(3,1,2), hold on
    xline(Parab_Periods(:), '--k')
    subplot(3,1,3), hold on
    xline(Parab_Periods(:), '--k')
    
    figure(82), hold on,
    subplot(3,1,1), hold on,
    plot(Tj_ext(:),Theta_r_ext(:),'*--k'), title("Traiettoria (Z)"), hold on
    xline(Parab_Periods(:), '--k')
    subplot(3,1,2), hold on
    xline(Parab_Periods(:), '--k')
    subplot(3,1,3), hold on
    xline(Parab_Periods(:), '--k')
    
    Tj_ext = [Tj(1)+tj(1)/2, Tj(2), Tj(3), Tj(4), Tj(5), Tj(6), Tj(7), Tj(8), Tj(9), Tj(10), Tj(11), Tj(12) - tj(n)/2]; % nel primo serve, nell'ultimo possiamo toglierlo
    
    j = 1; Tt = [];

    for i=1:n
    k = 1;
            if i == 1
                    tt = linspace(Tj(i), Tj(i+1)-tj(i+1)/2,1001);
                    for t = tt
                        if t >= Tj(i) && t <= Tj_ext(i) + tj(i)/2
                            tprimo = t-(Tj_ext(i)-tj(i)/2);
                            Theta_A(k) = Theta_j(1,i)+Thetapp_a(i)*tprimo^2/2;
                            Thetap_A(k) = Thetapp_a(i)*tprimo;
                            Thetapp_A(k) = Thetapp_a(i);
    
                            Theta_B(k) = Theta_j(2,i)+Thetapp_b(i)*tprimo^2/2;
                            Thetap_B(k) = Thetapp_b(i)*tprimo;
                            Thetapp_B(k) = Thetapp_b(i);
    
                            Theta_R(k) = Theta_j(3,i)+Thetapp_r(i)*tprimo^2/2;
                            Thetap_R(k) = Thetapp_r(i)*tprimo;
                            Thetapp_R(k) = Thetapp_r(i);
                        else
                            Theta_A(k) = Theta_j(1,i)+Thetap_a(i)*(t-Tj_ext(i));
                            Thetap_A(k) = Thetap_a(i);
                            Thetapp_A(k) = 0;
    
                            Theta_B(k) = Theta_j(2,i)+Thetap_b(i)*(t-Tj_ext(i));
                            Thetap_B(k) = Thetap_b(i);
                            Thetapp_B(k) = 0; 
    
                            Theta_R(k) = Theta_j(3,i)+Thetap_r(i)*(t-Tj_ext(i));
                            Thetap_R(k) = Thetap_r(i);
                            Thetapp_R(k) = 0; 
                        end
                        Theta(:,j) = [Theta_A(k) Theta_B(k) Theta_R(k)]';
                        Thetap(:,j) = [Thetap_A(k) Thetap_B(k) Thetap_R(k)]';
                        Thetapp(:,j) = [Thetapp_A(k) Thetapp_B(k) Thetapp_R(k)]';
                        k = k+1;
                        j = j+1;
                    end
                    Tt = [Tt tt];
                    
            elseif i ~= n
                    tt = linspace(Tj_ext(i)-tj(i)/2, Tj_ext(i+1)-tj(i+1)/2, 1001);
                    for t = tt
                        if t >= Tj_ext(i) - tj(i)/2 && t <= Tj_ext(i) + tj(i)/2 

                                tprimo = t-(Tj_ext(i)-tj(i)/2);
                                Theta_A(k) = Theta_j(1,i)-Thetap_a(i-1)*tj(i)/2+Thetap_a(i-1)*tprimo+Thetapp_a(i)*tprimo^2/2;
                                Thetap_A(k) = Thetap_a(i-1)+Thetapp_a(i)*tprimo;
                                Thetapp_A(k) = Thetapp_a(i);
    
                                Theta_B(k) = Theta_j(2,i)-Thetap_b(i-1)*tj(i)/2+Thetap_b(i-1)*tprimo+Thetapp_b(i)*tprimo^2/2;
                                Thetap_B(k) = Thetap_b(i-1)+Thetapp_b(i)*tprimo;
                                Thetapp_B(k) = Thetapp_b(i);
    
                                Theta_R(k) = Theta_j(3,i)-Thetap_r(i-1)*tj(i)/2+Thetap_r(i-1)*tprimo+Thetapp_r(i)*tprimo^2/2;
                                Thetap_R(k) = Thetap_r(i-1)+Thetapp_r(i)*tprimo;
                                Thetapp_R(k) = Thetapp_r(i);
                        else
                                Theta_A(k) = Theta_j(1,i)+Thetap_a(i)*(t-Tj_ext(i));
                                Thetap_A(k) = Thetap_a(i);
                                Thetapp_A(k) = 0;
    
                                Theta_B(k) = Theta_j(2,i)+Thetap_b(i)*(t-Tj_ext(i));
                                Thetap_B(k) = Thetap_b(i);
                                Thetapp_B(k) = 0;
    
                                Theta_R(k) = Theta_j(3,i)+Thetap_r(i)*(t-Tj_ext(i));
                                Thetap_R(k) = Thetap_r(i);
                                Thetapp_R(k) = 0;
                        end
                     Theta(:,j) = [Theta_A(k) Theta_B(k) Theta_R(k)]';
                     Thetap(:,j) = [Thetap_A(k) Thetap_B(k) Thetap_R(k)]';
                     Thetapp(:,j) = [Thetapp_A(k) Thetapp_B(k) Thetapp_R(k)]';
                     k = k+1;
                     j = j+1;
                    end
                    Tt = [Tt tt];
            else
                tt = linspace(Tj_ext(i)-tj(i)/2, Tj(i), 1001);
                for t = tt
                    if t >= Tj_ext(i)-tj(i)/2 && t <= Tj(i)    
                            tprimo = t-(Tj_ext(i)-tj(i)/2);
                            Theta_A(k) = Theta_j(1,i)-Thetap_a(i-1)*tj(i)/2+Thetap_a(i-1)*tprimo+Thetapp_a(i)*tprimo^2/2;
                            Thetap_A(k) = Thetap_a(i-1)+Thetapp_a(i)*tprimo;
                            Thetapp_A(k) = Thetapp_a(i);
    
                            Theta_B(k) = Theta_j(2,i)-Thetap_b(i-1)*tj(i)/2+Thetap_b(i-1)*tprimo+Thetapp_b(i)*tprimo^2/2;
                            Thetap_B(k) = Thetap_b(i-1)+Thetapp_b(i)*tprimo;
                            Thetapp_B(k) = Thetapp_b(i);
    
                            Theta_R(k) = Theta_j(3,i)-Thetap_r(i-1)*tj(i)/2+Thetap_r(i-1)*tprimo+Thetapp_r(i)*tprimo^2/2;
                            Thetap_R(k) = Thetap_r(i-1)+Thetapp_r(i)*tprimo;
                            Thetapp_R(k) = Thetapp_r(i);
                    else
                            Theta_A(k) = Theta_j(1,i)+Thetap_a(i)*(t-Tj_ext(i));
                            Thetap_A(k) = Thetap_a(i);
                            Thetapp_A(k) = 0;
    
                            Theta_B(k) = Theta_j(2,i)+Thetap_b(i)*(t-Tj_ext(i));
                            Thetap_B(k) = Thetap_b(i);
                            Thetapp_B(k) = 0;
    
                            Theta_R(k) = Theta_j(3,i)+Thetap_r(i)*(t-Tj_ext(i));
                            Thetap_R(k) = Thetap_r(i);
                            Thetapp_R(k) = 0;
                    end
                Theta(:,j) = [Theta_A(k) Theta_B(k) Theta_R(k)]';
                Thetap(:,j) = [Thetap_A(k) Thetap_B(k) Thetap_R(k)]';
                Thetapp(:,j) = [Thetapp_A(k) Thetapp_B(k) Thetapp_R(k)]';
                k = k+1;
                j = j+1;
                end
                Tt = [Tt tt];
            end
    end

  if error ~= 1 && Thetap_max(1) < Qp_max(1) && Thetap_max(2) < Qp_max(2) && Thetap_max(3) < Qp_max(3) && Thetapp_max(1) < Qpp_const(1) && Thetapp_max(2) < Qpp_const(2) && Thetapp_max(3) < Qpp_const(3) && isreal(tj)
        break;
    else
        coeff = coeff+0.5;
        error = 0;
    end
end %while
    figure(80)
    subplot(3,1,1)
    plot(Tt,Theta(1,:),'r-'), hold on
    xlabel('[s]'); ylabel('[rad]');
    subplot(3,1,2)
    plot(Tt,Thetap(1,:),'b'), hold on, title("Velocità (X)")
    xlabel('[s]'); ylabel('[rad/s]');
    subplot(3,1,3)
    plot(Tt,Thetapp(1,:),'b'), hold on, title("Accelerazioni (X)")
    xlabel('[s]'); ylabel('[rad/s^2]');

    figure(81)
    subplot(3,1,1)
    plot(Tt,Theta(2,:),'r-'), hold on
    xlabel('[s]'); ylabel('[rad]');
    subplot(3,1,2)
    plot(Tt,Thetap(2,:),'b'), hold on, title("Velocità (Y)")
    xlabel('[s]'); ylabel('[rad/s]');
    subplot(3,1,3)
    plot(Tt,Thetapp(2,:),'b'), hold on, title("Accelerazioni (Y)")
    xlabel('[s]'); ylabel('[rad/s^2]');
    
    figure(82)
    subplot(3,1,1)
    plot(Tt,Theta(3,:),'r-'), hold on
    xlabel('[s]'); ylabel('[m]');
    subplot(3,1,2)
    plot(Tt,Thetap(3,:),'b'), hold on, title("Velocità (Z)")
    xlabel('[s]'); ylabel('[m/s]');
    subplot(3,1,3)
    plot(Tt,Thetapp(3,:),'b'), hold on, title("Accelerazioni (Z)")
    xlabel('[s]'); ylabel('[m/s^2]');

end %della function